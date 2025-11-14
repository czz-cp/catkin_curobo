#!/usr/bin/env python3
"""
基于cuRobo MPC的视觉伺服控制器

改进点：
1. 使用cuRobo MPC替代简单比例控制，生成平滑、优化的轨迹
2. 考虑碰撞避免和关节限制
3. 实时位姿跟踪，支持动态目标
4. 更稳定的收敛性能
"""

import rospy
import numpy as np
import transforms3d as tf3d
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from sensor_msgs.msg import JointState as ROSJointState  # ROS标准消息类型
from std_msgs.msg import Float64MultiArray  # 用于关节速度控制
from tf2_ros import Buffer, TransformListener, TransformStamped
import tf2_geometry_msgs
import tf2_py as tf2
import torch

# cuRobo imports
try:
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.geom.types import WorldConfig
    from curobo.rollout.rollout_base import Goal
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import JointState, RobotConfig  # cuRobo的JointState类
    from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
    from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
    CUROBO_AVAILABLE = True
except ImportError as e:
    rospy.logwarn(f"cuRobo not available: {e}. Falling back to simple control.")
    CUROBO_AVAILABLE = False


class VisualServoMPCController:
    def __init__(self):
        rospy.init_node('visual_servo_mpc_controller')
        
        # 坐标系配置
        self.base_frame = "base"
        self.ee_frame = "tool0"  # 使用tool0作为末端执行器frame
        self.camera_frame = "camera_link"
        self.aruco_frame = "aruco_marker"
        
        # 控制参数
        self.error_threshold = 0.005  # 位置误差阈值 (m)
        self.angle_threshold = 0.02  # 角度误差阈值 (rad)
        self.mpc_step_dt = 0.03  # MPC时间步长 (s)
        self.control_rate = 30  # 控制频率 (Hz)
        
        # 期望的ArUco在相机坐标系下的位姿
        self.desired_position_in_cam = np.array([0.0, 0.0, 0.4])
        self.desired_euler_in_cam = np.array([np.pi, 0.0, 0.0])
        self.desired_quat_in_cam = tf3d.euler.euler2quat(
            self.desired_euler_in_cam[0],
            self.desired_euler_in_cam[1],
            self.desired_euler_in_cam[2],
            axes='sxyz'
        )
        
        # TF工具（增大缓存时间以处理延迟的ArUco消息）
        self.tf_buffer = Buffer(cache_time=rospy.Duration(20.0))  # 增加到20秒
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # ROS接口
        rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_callback)
        # 订阅关节状态（优先使用命名空间下的joint_states，这是真实的机器人状态）
        # UR机器人在命名空间下发布joint_states
        rospy.loginfo("订阅关节状态: /ur10e_robot/joint_states")
        self.joint_state_sub = rospy.Subscriber("/ur10e_robot/joint_states",
                                                ROSJointState,
                                                self.joint_state_callback)
        # 也订阅根命名空间的joint_states作为备选（可能是假的/joint_state_publisher）
        rospy.loginfo("订阅关节状态（备选）: /joint_states")
        self.joint_state_sub_root = rospy.Subscriber("/joint_states",
                                                     ROSJointState,
                                                     self.joint_state_callback)
        
        # 控制接口：使用关节速度控制器（更直接）
        # 注意：latch=True 确保发布器立即注册，即使没有订阅者
        self.joint_vel_pub = rospy.Publisher("/ur10e_robot/joint_group_vel_controller/command", 
                                            Float64MultiArray, queue_size=1, latch=False)
        # 保留Twist控制器作为备选（用于简单控制模式）
        self.cmd_pub = rospy.Publisher("/ur10e_robot/twist_controller/command", 
                                      Twist, queue_size=1)
        
        # 等待发布器注册（给ROS一些时间建立连接）
        rospy.sleep(0.5)
        
        # 当前关节状态
        self.current_joint_positions = None
        self.joint_names = None
        
        # ArUco检测有效性检查
        self.last_aruco_time = None  # 最后一次收到ArUco消息的时间
        self.aruco_timeout = 1.5  # ArUco超时时间（秒），如果超过这个时间没收到新消息就停止运动（放宽到1秒）
        self.min_aruco_distance = 0.05  # ArUco最小有效距离（米）（放宽）
        self.max_aruco_distance = 5.0  # ArUco最大有效距离（米）（放宽）
        
        # cuRobo MPC初始化
        self.mpc = None
        self.goal_buffer = None
        self.tensor_args = None
        self.target_pose_base = None
        
        if CUROBO_AVAILABLE:
            self._init_curobo_mpc()
        else:
            rospy.logwarn("使用简单比例控制器作为fallback")
            self.lambda_lin = 4.0
            self.lambda_ang = 4.0
            self.max_lin_vel = 3.0
            self.max_ang_vel = 1.5
        
        rospy.loginfo("基于cuRobo MPC的视觉伺服控制器已启动")
        rospy.loginfo(f"关节速度发布器: {self.joint_vel_pub.resolved_name}")
        rospy.loginfo(f"Twist发布器: {self.cmd_pub.resolved_name}")
        rospy.loginfo(f"ArUco超时检测: {self.aruco_timeout}秒（未检测到ArUco码时将停止运动）")
        
        # 启动ArUco超时检测定时器
        rospy.Timer(rospy.Duration(0.1), self._check_aruco_timeout)  # 每0.1秒检查一次
        
        # 检查cuRobo MPC是否成功初始化
        if self.mpc is None:
            rospy.logwarn("⚠ cuRobo MPC未初始化，将使用简单控制模式")
        else:
            rospy.loginfo("✓ cuRobo MPC已成功初始化")
        
        # 检查控制器是否运行（延迟检查，给启动脚本时间）
        rospy.Timer(rospy.Duration(12.0), self._check_controller_status, oneshot=True)
        
    def _init_curobo_mpc(self):
        """初始化cuRobo MPC求解器"""
        try:
            rospy.loginfo("初始化cuRobo MPC...")
            self.tensor_args = TensorDeviceType()
            
            # 加载机器人配置 (根据实际机器人修改)
            # 假设使用UR10e，需要对应的配置文件
            robot_file = "ur10e.yml"  # 或 ur5e.yml 根据实际情况
            
            try:
                robot_cfg = load_yaml(join_path(get_robot_configs_path(), robot_file))["robot_cfg"]
                robot_cfg = RobotConfig.from_dict(robot_cfg, self.tensor_args)
            except Exception as e:
                rospy.logerr(f"无法加载机器人配置 {robot_file}: {e}")
                rospy.logerr("请确保机器人配置文件存在于cuRobo配置目录中")
                # 重新抛出异常，让外层处理
                raise
            
            # 世界配置 (可选，用于碰撞避免)
            world_config = None  # 可以加载实际的障碍物配置
            
            # 创建MPC配置
            mpc_config = MpcSolverConfig.load_from_robot_config(
                robot_cfg,
                world_config,
                store_rollouts=False,  # 节省内存
                step_dt=self.mpc_step_dt,
            )
            
            self.mpc = MpcSolver(mpc_config)
            
            # 获取关节名称
            self.joint_names = self.mpc.joint_names
            self.robot_cfg = robot_cfg  # 保存robot_cfg用于雅可比计算
            
            rospy.loginfo(f"cuRobo MPC初始化成功，关节数: {len(self.joint_names)}")
            rospy.loginfo(f"关节名称: {self.joint_names}")
            
        except Exception as e:
            rospy.logerr(f"✗ cuRobo MPC初始化失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            rospy.logerr("cuRobo MPC初始化失败，关节速度控制将不可用")
            self.mpc = None
    
    def joint_state_callback(self, msg):
        """接收关节状态"""
        try:
            if hasattr(msg, 'position') and hasattr(msg, 'name'):
                # 确保关节顺序正确（UR10e通常有6个关节）
                positions = list(msg.position)
                names = list(msg.name)
                
                rospy.loginfo_throttle(2.0, f"✓ 收到关节状态: {len(positions)}个关节")
                rospy.loginfo_throttle(2.0, f"   关节名称: {names}")
                rospy.loginfo_throttle(2.0, f"   关节位置: {positions}")
                
                # 如果有cuRobo的关节名称，按顺序排列
                if self.joint_names is not None and len(self.joint_names) > 0:
                    ordered_positions = []
                    for joint_name in self.joint_names:
                        if joint_name in names:
                            idx = names.index(joint_name)
                            ordered_positions.append(positions[idx])
                        else:
                            rospy.logwarn_throttle(2.0, f"关节 {joint_name} 未在joint_states中找到")
                    if len(ordered_positions) == len(self.joint_names):
                        self.current_joint_positions = np.array(ordered_positions)
                        rospy.loginfo_throttle(1.0, f"✓ 关节状态已更新（按cuRobo顺序）: {self.current_joint_positions}")
                    else:
                        rospy.logwarn_throttle(2.0, f"关节数量不匹配: 期望{len(self.joint_names)}, 实际{len(ordered_positions)}")
                else:
                    # 如果没有预设顺序，直接使用前6个关节（假设是UR10e的6个关节）
                    if len(positions) >= 6:
                        self.current_joint_positions = np.array(positions[:6])
                        self.joint_names = names[:6] if len(names) >= 6 else None
                        rospy.loginfo_throttle(1.0, f"✓ 使用前6个关节: {names[:6] if len(names) >= 6 else 'unknown'}")
                        rospy.loginfo_throttle(1.0, f"✓ 关节状态已更新: {self.current_joint_positions}")
                    else:
                        rospy.logwarn_throttle(2.0, f"关节数量不足: 只有{len(positions)}个关节")
        except Exception as e:
            rospy.logerr_throttle(1.0, f"解析关节状态失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def get_current_joint_state(self):
        """获取当前关节状态"""
        # 检查关节状态是否已更新
        if self.current_joint_positions is None:
            rospy.logerr_throttle(1.0, "✗ 关节状态尚未接收！请检查/joint_states topic是否有数据")
            return None
        
        # 检查关节状态是否全为0（可能表示未初始化）
        if np.allclose(self.current_joint_positions, 0.0):
            rospy.logerr_throttle(1.0, f"✗ 错误: 关节位置全为0: {self.current_joint_positions}")
            rospy.logerr_throttle(1.0, "这可能表示:")
            rospy.logerr_throttle(1.0, "  1. 关节状态尚未正确初始化")
            rospy.logerr_throttle(1.0, "  2. 机器人未连接或处于初始位置")
            rospy.logerr_throttle(1.0, "  3. 订阅的joint_states topic不正确")
            rospy.logerr_throttle(1.0, "请检查: rostopic echo /joint_states")
            # 即使全为0也返回，让MPC尝试计算（可能是机器人真的在初始位置）
            return torch.from_numpy(self.current_joint_positions).float()
        
        if self.current_joint_positions is not None:
            return torch.from_numpy(self.current_joint_positions).float()
        
        # Fallback: 如果无法获取，返回零（需要实际实现查询）
        rospy.logerr("无法获取当前关节状态，使用零位置")
        return torch.zeros(6)  # 假设6自由度
    
    def compute_pbvs_velocity(self, aruco_pose_cam, stamp):
        """
        计算PBVS速度指令（在基座坐标系下）
        
        返回: v_cmd_base (6维数组: [vx, vy, vz, wx, wy, wz])
        """
        try:
            # 1. 计算相机坐标系下的误差（参考PBVS方法）
            pos_error_cam = np.array([
                self.desired_position_in_cam[0] - aruco_pose_cam.pose.position.x,
                self.desired_position_in_cam[1] - aruco_pose_cam.pose.position.y,
                self.desired_position_in_cam[2] - aruco_pose_cam.pose.position.z
            ])
            
            # 计算旋转误差
            q_current = np.array([
                aruco_pose_cam.pose.orientation.w,
                aruco_pose_cam.pose.orientation.x,
                aruco_pose_cam.pose.orientation.y,
                aruco_pose_cam.pose.orientation.z
            ])
            q_current_inv = tf3d.quaternions.qinverse(q_current)
            q_error = tf3d.quaternions.qmult(self.desired_quat_in_cam, q_current_inv)
            
            # 转换为轴角表示（旋转向量）
            angle = 2 * np.arccos(np.clip(q_error[0], -1.0, 1.0))
            if np.abs(angle) < 1e-6:
                rot_error_cam = np.zeros(3)
            else:
                axis = q_error[1:] / np.linalg.norm(q_error[1:])
                rot_error_cam = angle * axis
            
            error_cam = np.concatenate([pos_error_cam, rot_error_cam])
            
            # 2. 在相机坐标系下生成速度指令（参考PBVS方法）
            # 调整增益，使速度响应适中，避免震荡
            lambda_lin =10.0  # 线速度增益 4.0
            lambda_ang =10.0  # 角速度增益 4.0
            v_cmd_cam = np.zeros(6)
            v_cmd_cam[:3] = -lambda_lin * error_cam[:3]  # 线速度
            v_cmd_cam[3:] = -lambda_ang * error_cam[3:]  # 角速度
            
            # 速度限幅（适中的限幅，避免过大或过小）
            max_lin_vel = 5.0  # 线速度限制（m/s） 2.0
            max_ang_vel = 3.0  # 角速度限制（rad/s） 1.5
            v_cmd_cam[:3] = np.clip(v_cmd_cam[:3], -max_lin_vel, max_lin_vel)
            v_cmd_cam[3:] = np.clip(v_cmd_cam[3:], -max_ang_vel, max_ang_vel)
            
            # 3. 将速度从相机坐标系转换到末端坐标系（使用伴随矩阵）
            # 使用最新时间查询TF，而不是消息的时间戳（可能太旧）
            tf_time = rospy.Time(0)  # 使用最新可用的TF变换
            cam_to_ee_tf = self.get_transform(self.ee_frame, self.camera_frame, tf_time)
            rotation = cam_to_ee_tf.transform.rotation
            R_cam_to_ee = tf3d.quaternions.quat2mat([
                rotation.w, rotation.x, rotation.y, rotation.z
            ])
            translation = cam_to_ee_tf.transform.translation
            t_cam_to_ee = np.array([translation.x, translation.y, translation.z])
            
            # 构造伴随矩阵
            skew_sym = np.array([
                [0, -t_cam_to_ee[2], t_cam_to_ee[1]],
                [t_cam_to_ee[2], 0, -t_cam_to_ee[0]],
                [-t_cam_to_ee[1], t_cam_to_ee[0], 0]
            ])
            skew_R = skew_sym @ R_cam_to_ee
            
            adjoint = np.zeros((6, 6))
            adjoint[:3, :3] = R_cam_to_ee
            adjoint[:3, 3:] = skew_R
            adjoint[3:, 3:] = R_cam_to_ee
            
            v_cmd_ee = adjoint @ v_cmd_cam
            
            # 4. 将速度从末端坐标系转换到基座坐标系
            ee_to_base_tf = self.get_transform(self.base_frame, self.ee_frame, tf_time)
            rotation = ee_to_base_tf.transform.rotation
            R_ee_to_base = tf3d.quaternions.quat2mat([
                rotation.w, rotation.x, rotation.y, rotation.z
            ])
            
            v_cmd_base = np.zeros(6)
            v_cmd_base[:3] = R_ee_to_base @ v_cmd_ee[:3]  # 线速度
            v_cmd_base[3:] = R_ee_to_base @ v_cmd_ee[3:]  # 角速度
            
            # 保存速度指令供直接使用（绕过MPC）
            self.v_cmd_base = v_cmd_base  # 保存基座坐标系下的速度
            
            rospy.loginfo_throttle(0.5, 
                f"视觉伺服计算（PBVS方法）:")
            rospy.loginfo_throttle(0.5, 
                f"  ArUco检测位置(相机坐标系): [{aruco_pose_cam.pose.position.x:.4f}, "
                f"{aruco_pose_cam.pose.position.y:.4f}, {aruco_pose_cam.pose.position.z:.4f}]")
            rospy.loginfo_throttle(0.5, 
                f"  期望位置(相机坐标系): {self.desired_position_in_cam}")
            rospy.loginfo_throttle(0.5, 
                f"  相机坐标系误差: 位置={pos_error_cam}, 旋转={rot_error_cam}")
            rospy.loginfo_throttle(0.5, 
                f"  相机坐标系速度: 线速度={v_cmd_cam[:3]}, 角速度={v_cmd_cam[3:]}")
            rospy.loginfo_throttle(0.5, 
                f"  基座坐标系速度: 线速度={v_cmd_base[:3]}, 角速度={v_cmd_base[3:]}")
            
            return v_cmd_base
            
        except Exception as e:
            rospy.logerr(f"计算PBVS速度失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None
    
    
    def aruco_callback(self, msg):
        """ArUco位姿回调 - 主控制循环"""
        rospy.loginfo_throttle(2.0, "=== aruco_callback 被调用 ===")
        try:
            if msg is None:
                rospy.logwarn_throttle(1.0, "接收到空的ArUco消息")
                self.publish_zero_velocity()
                return
            
            # 检查ArUco位置的有效性（在相机坐标系下）
            aruco_pos_cam = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            aruco_distance = np.linalg.norm(aruco_pos_cam)
            
            # 检查距离是否在有效范围内
            if aruco_distance < self.min_aruco_distance or aruco_distance > self.max_aruco_distance:
                rospy.logwarn_throttle(1.0, 
                    f"ArUco距离无效: {aruco_distance:.3f}m (有效范围: {self.min_aruco_distance}-{self.max_aruco_distance}m)，停止运动")
                self.publish_zero_velocity()
                return
            
            # 检查位置是否全为0（可能是无效检测）
            if np.allclose(aruco_pos_cam, 0, atol=1e-6):
                rospy.logwarn_throttle(1.0, "ArUco位置全为0，可能是无效检测，停止运动")
                self.publish_zero_velocity()
                return
            
            # 更新最后一次收到有效ArUco消息的时间
            self.last_aruco_time = rospy.Time.now()
            
            rospy.loginfo_throttle(1.0, f"✓ 接收到有效ArUco消息，距离: {aruco_distance:.3f}m")
            
            # 检查cuRobo是否可用
            if self.mpc is None or not CUROBO_AVAILABLE:
                rospy.logerr_throttle(1.0, "✗ cuRobo MPC不可用！无法使用关节速度控制")
                rospy.logerr_throttle(1.0, "   请检查cuRobo是否正确安装和配置")
                rospy.logerr_throttle(1.0, "   当前将发布零速度")
                self.publish_zero_velocity()
                return
            
            rospy.loginfo_throttle(1.0, "使用MPC控制模式")
            
            # 1. 先获取当前关节状态（用于计算当前末端位姿）
            current_q = self.get_current_joint_state()
            if current_q is None:
                rospy.logerr_throttle(1.0, "✗ 无法获取当前关节状态，发布零速度")
                self.publish_zero_velocity()
                return
            
            current_joints_np = current_q.cpu().numpy() if isinstance(current_q, torch.Tensor) else current_q
            
            # 严格检查关节位置是否有效（避免全0或其他异常值）
            if np.allclose(current_joints_np, 0, atol=1e-6):
                rospy.logerr_throttle(1.0, "✗ 错误: 关节位置全为0，跳过本次计算")
                self.publish_zero_velocity()
                return
            
            # 检查关节位置是否在合理范围内（UR10e的关节限位大约是±6.28弧度）
            if np.any(np.abs(current_joints_np) > 10.0):
                rospy.logerr_throttle(1.0, f"✗ 错误: 关节位置超出合理范围: {current_joints_np}")
                self.publish_zero_velocity()
                return
            
            rospy.loginfo_throttle(1.0, f"✓ 当前关节位置: {current_joints_np}")
            
            # 2. 创建current_state并计算当前末端位姿（使用MPC的正运动学）
            current_state = JointState.from_position(
                current_q.unsqueeze(0).to(self.tensor_args.device),
                joint_names=self.joint_names
            )
            
            # 使用MPC计算当前末端位姿（确保一致性）
            current_ee_state = self.mpc.rollout_fn.compute_kinematics(current_state)
            current_ee_pos_from_mpc = current_ee_state.ee_pos_seq[0].cpu().numpy()
            current_ee_quat_from_mpc = current_ee_state.ee_quat_seq[0].cpu().numpy()
            
            rospy.loginfo_throttle(0.5, 
                f"MPC当前末端位姿（从关节计算）: 位置={current_ee_pos_from_mpc}, "
                f"四元数={current_ee_quat_from_mpc}")
            
            # 3. 计算期望的最终位姿（基于ArUco期望位置转换到基座坐标系）
            # 这是视觉伺服的目标：让ArUco在相机坐标系中处于期望位置
            tf_time = rospy.Time(0)
            
            # 3.1 计算期望的ArUco位置在基座坐标系中的位姿
            # 期望位置在相机坐标系中：desired_position_in_cam
            # 需要转换到基座坐标系
            cam_to_base_tf = self.get_transform(self.base_frame, self.camera_frame, tf_time)
            cam_rot = cam_to_base_tf.transform.rotation
            R_cam_to_base = tf3d.quaternions.quat2mat([
                cam_rot.w, cam_rot.x, cam_rot.y, cam_rot.z
            ])
            cam_trans = cam_to_base_tf.transform.translation
            t_cam_to_base = np.array([cam_trans.x, cam_trans.y, cam_trans.z])
            
            # 期望ArUco位置在相机坐标系中
            desired_aruco_pos_cam = np.array(self.desired_position_in_cam)
            
            # 转换到基座坐标系
            desired_aruco_pos_base = R_cam_to_base @ desired_aruco_pos_cam + t_cam_to_base
            
            # 期望ArUco四元数在相机坐标系中
            desired_aruco_quat_cam = np.array(self.desired_quat_in_cam)
            
            # 转换到基座坐标系：使用四元数乘法
            q_cam_to_base = np.array([cam_rot.w, cam_rot.x, cam_rot.y, cam_rot.z])
            desired_aruco_quat_base = tf3d.quaternions.qmult(q_cam_to_base, desired_aruco_quat_cam)
            
            # 3.2 计算当前ArUco位置在基座坐标系中的位姿
            current_aruco_pos_cam = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            current_aruco_pos_base = R_cam_to_base @ current_aruco_pos_cam + t_cam_to_base
            
            current_aruco_quat_cam = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            current_aruco_quat_base = tf3d.quaternions.qmult(q_cam_to_base, current_aruco_quat_cam)
            
            # 3.3 计算期望的末端执行器位姿
            # 关键理解：视觉伺服的目标是让ArUco在相机坐标系中处于期望位置
            # 如果ArUco在相机坐标系中的期望位置是 desired_position_in_cam
            # 当前ArUco在相机坐标系中的位置是 current_aruco_pos_cam
            # 那么相机需要移动，使得误差为0
            
            # 计算相机坐标系中的误差
            error_in_cam = desired_aruco_pos_cam - current_aruco_pos_cam
            
            # 相机需要移动（在相机坐标系中）：-error_in_cam（让误差为0）
            # 转换到基座坐标系
            camera_move_in_base = R_cam_to_base @ (-error_in_cam)
            
            # 期望末端位置 = 当前末端位置 + 相机移动（因为末端和相机是刚体连接）
            desired_ee_pos_base = current_ee_pos_from_mpc + camera_move_in_base
            
            # 对于旋转，计算旋转误差
            q_current_aruco_cam = current_aruco_quat_cam
            q_desired_aruco_cam = desired_aruco_quat_cam
            q_rot_error_cam = tf3d.quaternions.qmult(q_desired_aruco_cam, tf3d.quaternions.qinverse(q_current_aruco_cam))
            
            # 将旋转误差从相机坐标系转换到基座坐标系
            # 旋转误差在相机坐标系中，需要转换到基座坐标系
            # q_rot_error_base = cam_to_base旋转 * q_rot_error_cam * cam_to_base旋转的逆
            q_rot_error_base = tf3d.quaternions.qmult(
                q_cam_to_base,
                tf3d.quaternions.qmult(q_rot_error_cam, tf3d.quaternions.qinverse(q_cam_to_base))
            )
            
            # 期望末端旋转 = 当前末端旋转 * 旋转误差（基座坐标系）
            desired_ee_quat_base = tf3d.quaternions.qmult(current_ee_quat_from_mpc, q_rot_error_base)
            
            # 3.4 计算当前位姿到期望位姿的距离
            pos_error_to_final = np.linalg.norm(desired_ee_pos_base - current_ee_pos_from_mpc)
            
            # 3.5 如果距离较远，使用渐进式目标（向期望位置移动，但限制单次移动距离）
            # 如果距离较近，直接使用期望位置作为目标
            # 增大单次移动距离，提高跟踪速度
            max_single_step = 0.4  # 单次最大移动距离（米），从0.2增加到0.4以提高跟踪速度
            
            # 如果误差很大，使用更大的步长
            if pos_error_to_final > 0.5:
                max_single_step = 0.6  # 误差很大时，使用更大的步长
            
            if pos_error_to_final > max_single_step:
                # 距离较远，向期望位置移动，但限制单次移动距离
                direction_to_final = (desired_ee_pos_base - current_ee_pos_from_mpc) / pos_error_to_final
                target_pos_base = current_ee_pos_from_mpc + direction_to_final * max_single_step
                rospy.loginfo_throttle(1.0, 
                    f"距离最终目标较远({pos_error_to_final:.3f}m)，使用渐进式目标，移动{max_single_step}m")
            else:
                # 距离较近，直接使用期望位置
                target_pos_base = desired_ee_pos_base
                rospy.loginfo_throttle(1.0, 
                    f"距离最终目标较近({pos_error_to_final:.3f}m)，直接使用期望位置")
            
            # 对于旋转，使用类似的方法
            # 计算旋转误差
            q_current_inv = tf3d.quaternions.qinverse(current_ee_quat_from_mpc)
            q_rot_error = tf3d.quaternions.qmult(desired_ee_quat_base, q_current_inv)
            
            # 转换为轴角
            angle_error = 2 * np.arccos(np.clip(q_rot_error[0], -1.0, 1.0))
            max_angle_step = 0.5  # 单次最大旋转角度（弧度），从0.3增加到0.5以提高跟踪速度
            
            # 如果旋转误差很大，使用更大的角度步长
            if angle_error > 1.0:
                max_angle_step = 0.8
            
            if angle_error > max_angle_step:
                # 旋转误差较大，使用渐进式旋转
                axis = q_rot_error[1:] / np.linalg.norm(q_rot_error[1:])
                q_delta = np.array([
                    np.cos(max_angle_step / 2.0),
                    np.sin(max_angle_step / 2.0) * axis[0],
                    np.sin(max_angle_step / 2.0) * axis[1],
                    np.sin(max_angle_step / 2.0) * axis[2]
                ])
                target_quat_base = tf3d.quaternions.qmult(current_ee_quat_from_mpc, q_delta)
            else:
                # 旋转误差较小，直接使用期望旋转
                target_quat_base = desired_ee_quat_base
            
            # 计算目标位姿与当前位姿的差异（用于调试）
            pos_diff = np.linalg.norm(target_pos_base - current_ee_pos_from_mpc)
            
            rospy.loginfo_throttle(0.5, 
                f"MPC目标位姿: 位置={target_pos_base}, 四元数={target_quat_base}")
            rospy.loginfo_throttle(0.5, 
                f"位置差异（手动计算）: {pos_diff:.4f}m, "
                f"当前位姿={current_ee_pos_from_mpc}, 目标位姿={target_pos_base}")
            
            # 创建目标位姿（确保形状正确）
            target_pos_tensor = torch.from_numpy(target_pos_base).float().unsqueeze(0).to(self.tensor_args.device)
            target_quat_tensor = torch.from_numpy(target_quat_base).float().unsqueeze(0).to(self.tensor_args.device)
            
            # 验证tensor形状
            if target_pos_tensor.shape != (1, 3):
                rospy.logerr(f"✗ 目标位置tensor形状错误: {target_pos_tensor.shape}, 期望 (1, 3)")
            if target_quat_tensor.shape != (1, 4):
                rospy.logerr(f"✗ 目标四元数tensor形状错误: {target_quat_tensor.shape}, 期望 (1, 4)")
            
            target_pose = Pose(
                position=target_pos_tensor,
                quaternion=target_quat_tensor
            )
            
            # 验证创建的Pose对象
            rospy.loginfo_throttle(0.5, 
                f"创建的Pose对象: 位置={target_pose.position[0].cpu().numpy()}, "
                f"四元数={target_pose.quaternion[0].cpu().numpy()}")
            
            # 创建goal_state：使用current_state，但MPC会优先使用goal_pose
            goal = Goal(
                current_state=current_state,
                goal_state=current_state,  # MPC主要使用goal_pose，goal_state仅作为参考
                goal_pose=target_pose,
            )
            
            # 4. 运行MPC
            try:
                # 每次都需要重新创建Goal对象并调用setup_solve_single
                # 因为update_goal可能会改变目标位姿，直接创建新的goal_buffer更可靠
                self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
                
                # 验证goal_buffer中的目标位姿
                goal_pose_in_mpc = self.goal_buffer.goal_pose
                goal_pos_after_setup = goal_pose_in_mpc.position[0].cpu().numpy()
                goal_quat_after_setup = goal_pose_in_mpc.quaternion[0].cpu().numpy()
                
                rospy.loginfo_throttle(0.5, 
                    f"MPC内部目标位姿（setup_solve_single后）: 位置={goal_pos_after_setup}, "
                    f"四元数={goal_quat_after_setup}")
                
                # 检查目标位姿是否被setup_solve_single改变了
                pos_diff_after_setup = np.linalg.norm(target_pos_base - goal_pos_after_setup)
                if pos_diff_after_setup > 0.01:  # 如果差异超过1cm，发出警告
                    rospy.logwarn_throttle(1.0, 
                        f"⚠ 警告：setup_solve_single改变了目标位姿！差异={pos_diff_after_setup:.4f}m")
                    rospy.logwarn_throttle(1.0,
                        f"   设置的目标: {target_pos_base}, MPC内部: {goal_pos_after_setup}")
                
                # 使用setup_solve_single后的目标位姿（确保一致性）
                goal_pos_after_update = goal_pos_after_setup
                goal_quat_after_update = goal_quat_after_setup
                
                # 重要：根据cuRobo示例代码，在step之前需要调用update_goal
                # 这确保MPC使用最新的目标位姿
                self.mpc.update_goal(self.goal_buffer)
                
                result = self.mpc.step(current_state, 1)
                torch.cuda.synchronize()
                # 打印MPC计算的当前末端位姿（从result中获取）
                result_ee_state = self.mpc.rollout_fn.compute_kinematics(result.action)
                result_ee_pos = result_ee_state.ee_pos_seq[0].cpu().numpy()
                result_ee_quat = result_ee_state.ee_quat_seq[0].cpu().numpy()
                
                # 计算MPC输出的关节速度（用于调试）
                action_joints = result.action.position[0].cpu().numpy()
                current_joints_np = current_state.position[0].cpu().numpy()
                joint_vel_from_mpc = (action_joints - current_joints_np) / self.mpc_step_dt
                joint_vel_magnitude = np.linalg.norm(joint_vel_from_mpc)
                
                rospy.loginfo_throttle(0.5,
                    f"MPC计算的动作后末端位姿: 位置={result_ee_pos}, 四元数={result_ee_quat}")
                rospy.loginfo_throttle(0.5,
                    f"MPC输出的关节速度幅值: {joint_vel_magnitude:.4f} rad/s")
                rospy.loginfo_throttle(0.5,
                    f"MPC输出的关节速度: {joint_vel_from_mpc}")
                
                rospy.loginfo_throttle(1.0, "✓ MPC计算成功")
            except Exception as e:
                rospy.logerr(f"✗ MPC计算失败: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
                self.publish_zero_velocity()
                return
            
            # 5. 检查收敛（使用手动计算的位置误差，而不是MPC的pose_error）
            # MPC的pose_error可能包含旋转误差，单位可能有问题
            # 使用update_goal之后的目标位姿
            goal_pos_mpc = goal_pos_after_update
            goal_quat_mpc = goal_quat_after_update
            manual_pos_error = np.linalg.norm(result_ee_pos - goal_pos_mpc)
            
            # 计算旋转误差（四元数角度差）
            q_current = result_ee_quat
            q_target = goal_quat_mpc
            q_error = tf3d.quaternions.qmult(q_target, tf3d.quaternions.qinverse(q_current))
            angle_error = 2 * np.arccos(np.clip(np.abs(q_error[0]), 0, 1.0))
            
            rospy.loginfo_throttle(0.5, 
                f"MPC报告姿态误差: {result.metrics.pose_error.item():.4f}m (可能包含旋转误差)")
            rospy.loginfo_throttle(0.5,
                f"手动计算误差: 位置={manual_pos_error:.4f}m, 旋转={angle_error:.4f}rad")
            
            # 使用手动计算的位置误差来判断收敛
            if manual_pos_error < self.error_threshold and angle_error < self.angle_threshold:
                rospy.loginfo_throttle(1.0, "已达到目标精度")
                self.publish_zero_velocity()
                return
            
            # 6. 发布控制指令
            # 完全使用MPC输出，不切换到PBVS直接方法
            # 优先使用MPC的velocity输出（这是MPC直接计算的速度，更准确）
            if hasattr(result.action, 'velocity') and result.action.velocity is not None:
                joint_velocities = result.action.velocity[0].cpu().numpy()
                rospy.loginfo_throttle(0.5, "✓ 使用MPC计算的velocity输出")
                rospy.loginfo_throttle(0.5, f"MPC velocity输出: {joint_velocities}, 幅值: {np.linalg.norm(joint_velocities):.4f} rad/s")
            else:
                # 否则使用位置差分计算速度（基于MPC计算的关节速度）
                # 使用上面已经计算的joint_vel_from_mpc
                joint_velocities = joint_vel_from_mpc
                rospy.loginfo_throttle(0.5, "⚠ 使用位置差分计算速度（MPC未提供velocity）")
                rospy.loginfo_throttle(0.5, f"位置差分计算的速度: {joint_velocities}, 幅值: {joint_vel_magnitude:.4f} rad/s")
            
            # 如果MPC输出的速度太小，放大它以产生足够的控制力
            # 同时，如果位置误差较大，进一步放大速度
            vel_magnitude_before = np.linalg.norm(joint_velocities)
            min_vel_threshold = 0.8  # 如果速度小于0.8 rad/s，认为太小（提高阈值以提高跟踪速度）
            vel_scale_factor = 1.0
            
            if vel_magnitude_before > 0:
                # 基础放大：如果速度太小，放大到至少0.8 rad/s
                if vel_magnitude_before < min_vel_threshold:
                    base_scale = min_vel_threshold / vel_magnitude_before
                else:
                    base_scale = 1.0
                
                # 基于位置误差的额外放大：如果位置误差大（>0.05m），进一步放大
                if manual_pos_error > 0.05:
                    # 位置误差越大，放大倍数越大（最多15倍，提高跟踪速度）
                    error_scale = min(1.0 + (manual_pos_error - 0.05) * 30.0, 15.0)
                else:
                    error_scale = 1.0
                
                # 总放大倍数（不超过30倍，允许更大的放大以提高跟踪速度）
                vel_scale_factor = min(base_scale * error_scale, 30.0)
                joint_velocities = joint_velocities * vel_scale_factor
                
                if vel_scale_factor > 1.1:  # 只有放大超过10%才记录
                    rospy.logwarn_throttle(1.0, 
                        f"⚠ MPC速度放大: 原始={vel_magnitude_before:.4f} rad/s, "
                        f"位置误差={manual_pos_error:.3f}m, "
                        f"放大{vel_scale_factor:.2f}倍到{np.linalg.norm(joint_velocities):.4f} rad/s")
            
            # 限幅（大幅提高最大速度，让机器人能更快响应跟踪）
            max_joint_vel = 4.0  # 提高最大关节速度到4.0 rad/s以提高跟踪速度
            joint_velocities = np.clip(joint_velocities, -max_joint_vel, max_joint_vel)
            
            # 如果位置误差仍然较大，进一步增大速度
            if manual_pos_error > 0.05:
                # 确保速度至少达到1.2 rad/s（如果位置误差大，大幅提高最小速度以提高跟踪速度）
                vel_final = np.linalg.norm(joint_velocities)
                if vel_final < 1.2:
                    additional_scale = 1.2 / vel_final if vel_final > 0 else 1.0
                    joint_velocities = joint_velocities * additional_scale
                    joint_velocities = np.clip(joint_velocities, -max_joint_vel, max_joint_vel)
                    rospy.logwarn_throttle(1.0,
                        f"⚠ 位置误差较大({manual_pos_error:.3f}m)，进一步放大速度到{np.linalg.norm(joint_velocities):.3f} rad/s")
            
            # 打印关节速度信息
            self._print_joint_velocities(joint_velocities)
            
            # 检查关节速度是否非零
            vel_magnitude = np.linalg.norm(joint_velocities)
            if vel_magnitude < 1e-6:
                rospy.logwarn_throttle(1.0, "⚠ 关节速度接近零，机械臂可能不动")
                rospy.logwarn_throttle(1.0, f"   关节速度: {joint_velocities}")
            else:
                rospy.loginfo_throttle(0.5, f"✓ 关节速度幅值: {vel_magnitude:.4f} rad/s")
                rospy.loginfo_throttle(0.5, f"   关节速度: {joint_velocities}")
            
            # 直接发布关节速度（更直接、更高效）
            rospy.loginfo_throttle(0.5, "正在发布关节速度命令...")
            self._publish_joint_velocities(joint_velocities)
            
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"TF错误: {str(e)}")
            self.publish_zero_velocity()
        except Exception as e:
            rospy.logerr_throttle(1.0, f"MPC控制错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            self.publish_zero_velocity()
    
    def _simple_control(self, msg):
        """简单的比例控制器作为fallback"""
        try:
            error_in_cam = self.compute_error_in_cam(msg)
            
            pos_error_magnitude = np.linalg.norm(error_in_cam[:3])
            rot_error_magnitude = np.linalg.norm(error_in_cam[3:])
            
            if pos_error_magnitude < self.error_threshold and rot_error_magnitude < self.angle_threshold:
                self.publish_zero_velocity()
                return
            
            twist_cam = self.generate_camera_twist(error_in_cam)
            twist_ee = self.transform_twist_to_ee(twist_cam, msg.header.stamp)
            twist_base = self.transform_twist_to_base(twist_ee, msg.header.stamp)
            
            # 打印速度信息
            self._print_twist_velocity(twist_base, "简单控制")
            
            self.cmd_pub.publish(twist_base)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"简单控制错误: {e}")
            self.publish_zero_velocity()
    
    def _joint_velocities_to_twist(self, joint_velocities, current_state, stamp):
        """将关节速度转换为末端执行器的Twist"""
        twist = Twist()
        
        try:
            if self.mpc is None or not CUROBO_AVAILABLE:
                rospy.logwarn_throttle(1.0, "cuRobo MPC不可用，无法计算雅可比矩阵")
                return twist
            
            # 方法1: 尝试使用cuRobo的kinematics计算雅可比
            # 将关节速度转换为torch tensor
            joint_vel_tensor = torch.from_numpy(joint_velocities).float().to(self.tensor_args.device)
            
            try:
                # 使用cuRobo的kinematics计算末端位姿，然后使用数值微分
                # 获取当前和下一个关节位置
                current_q = current_state.position[0]  # (6,)
                dt = 0.001  # 小时间步用于数值微分
                next_q = current_q + joint_vel_tensor * dt
                
                # 计算当前和下一个末端位姿
                current_pose = self.mpc.compute_kinematics(current_state)
                next_state = JointState.from_position(
                    next_q.unsqueeze(0).to(self.tensor_args.device),
                    joint_names=self.joint_names
                )
                next_pose = self.mpc.compute_kinematics(next_state)
                
                # 计算末端速度（数值微分）
                pos_diff = (next_pose.position[0] - current_pose.position[0]).cpu().numpy()
                quat_diff = next_pose.quaternion[0].cpu().numpy() - current_pose.quaternion[0].cpu().numpy()
                
                # 计算线速度
                twist.linear.x = pos_diff[0] / dt
                twist.linear.y = pos_diff[1] / dt
                twist.linear.z = pos_diff[2] / dt
                
                # 计算角速度（从四元数差分）
                # 简化处理：使用四元数差分近似角速度
                q_current = current_pose.quaternion[0].cpu().numpy()
                q_next = next_pose.quaternion[0].cpu().numpy()
                # q_error = q_next * q_current^-1
                q_current_inv = tf3d.quaternions.qinverse(q_current)
                q_error = tf3d.quaternions.qmult(q_next, q_current_inv)
                
                # 四元数到角速度
                angle = 2 * np.arccos(np.clip(q_error[0], -1.0, 1.0))
                if angle > 1e-6:
                    axis = q_error[1:] / np.linalg.norm(q_error[1:])
                    angular_vel = (angle / dt) * axis
                else:
                    angular_vel = np.zeros(3)
                
                twist.angular.x = angular_vel[0]
                twist.angular.y = angular_vel[1]
                twist.angular.z = angular_vel[2]
                
            except Exception as e:
                # 方法2: 使用TF变换和数值微分（备选方案）
                rospy.logwarn_throttle(2.0, f"使用cuRobo计算失败，尝试TF方法: {e}")
                try:
                    # 获取当前末端位姿（使用最新时间，避免时间戳过旧的问题）
                    ee_tf = self.get_transform(self.base_frame, self.ee_frame, rospy.Time(0))
                    current_pos = np.array([
                        ee_tf.transform.translation.x,
                        ee_tf.transform.translation.y,
                        ee_tf.transform.translation.z
                    ])
                    current_quat = np.array([
                        ee_tf.transform.rotation.w,
                        ee_tf.transform.rotation.x,
                        ee_tf.transform.rotation.y,
                        ee_tf.transform.rotation.z
                    ])
                    
                    # 计算小位移后的位姿（使用关节速度近似）
                    # 这里简化处理，直接使用关节速度的线性组合
                    # 实际应该使用雅可比矩阵
                    rospy.logwarn_throttle(2.0, "使用简化方法计算末端速度，精度可能较低")
                    # 暂时返回关节速度的某种映射（需要实际的运动学模型）
                    # 这里先返回零，让用户知道需要实现
                    pass
                except Exception as e2:
                    rospy.logerr_throttle(1.0, f"所有方法都失败: {e2}")
                
        except Exception as e:
            rospy.logerr_throttle(1.0, f"关节速度转Twist失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
        
        return twist
    
    def _publish_joint_velocities(self, joint_velocities):
        """发布关节速度指令到joint_group_vel_controller"""
        try:
            # 检查关节速度是否有效
            if joint_velocities is None or len(joint_velocities) == 0:
                rospy.logwarn("关节速度为空，无法发布")
                return
            
            # 检查关节数量是否匹配
            if self.joint_names is not None and len(joint_velocities) != len(self.joint_names):
                rospy.logwarn_throttle(1.0, 
                    f"关节速度数量不匹配: 期望{len(self.joint_names)}，实际{len(joint_velocities)}")
            
            # 创建Float64MultiArray消息
            joint_vel_msg = Float64MultiArray()
            joint_vel_msg.data = joint_velocities.tolist()
            
            # 检查发布者是否已连接
            num_subscribers = self.joint_vel_pub.get_num_connections()
            if num_subscribers == 0:
                rospy.logerr_throttle(1.0, 
                    f"✗ 错误: joint_group_vel_controller没有订阅者！控制器可能未启动。"
                    f"Topic: {self.joint_vel_pub.resolved_name}")
                rospy.logerr_throttle(1.0, "请在rqt中启动joint_group_vel_controller")
            else:
                rospy.loginfo_throttle(0.5, 
                    f"✓ 发布关节速度到 {num_subscribers} 个订阅者")
                rospy.loginfo_throttle(0.5, f"   速度值: {joint_velocities}")
            
            # 发布关节速度
            self.joint_vel_pub.publish(joint_vel_msg)
            rospy.loginfo_throttle(0.5, "✓ 关节速度消息已发布")
            
        except Exception as e:
            rospy.logerr_throttle(1.0, f"发布关节速度失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            # Fallback: 发布零速度
            self.publish_zero_velocity()
    
    def _send_joint_velocities(self, joint_velocities):
        """发送关节速度指令（已弃用，使用_publish_joint_velocities代替）"""
        self._publish_joint_velocities(joint_velocities)
    
    def compute_error_in_cam(self, aruco_pose_cam):
        """计算ArUco在相机坐标系下的位姿误差（用于简单控制）"""
        pos_error = np.array([
            self.desired_position_in_cam[0] - aruco_pose_cam.pose.position.x,
            self.desired_position_in_cam[1] - aruco_pose_cam.pose.position.y,
            self.desired_position_in_cam[2] - aruco_pose_cam.pose.position.z
        ])
        
        q_current = np.array([
            aruco_pose_cam.pose.orientation.w,
            aruco_pose_cam.pose.orientation.x,
            aruco_pose_cam.pose.orientation.y,
            aruco_pose_cam.pose.orientation.z
        ])
        
        q_current_inv = tf3d.quaternions.qinverse(q_current)
        q_error = tf3d.quaternions.qmult(self.desired_quat_in_cam, q_current_inv)
        
        angle = 2 * np.arccos(np.clip(q_error[0], -1.0, 1.0))
        if np.abs(angle) < 1e-6:
            rot_error = np.zeros(3)
        else:
            axis = q_error[1:] / np.linalg.norm(q_error[1:])
            rot_error = angle * axis
        
        return np.concatenate([pos_error, rot_error])
    
    def generate_camera_twist(self, error):
        """生成相机坐标系下的Twist指令"""
        v_cmd = np.zeros(6)
        v_cmd[:3] = -self.lambda_lin * error[:3]
        v_cmd[3:] = -self.lambda_ang * error[3:]
        
        lin_vel = np.clip(v_cmd[:3], -self.max_lin_vel, self.max_lin_vel)
        ang_vel = np.clip(v_cmd[3:], -self.max_ang_vel, self.max_ang_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel[0]
        twist_msg.linear.y = lin_vel[1]
        twist_msg.linear.z = lin_vel[2]
        twist_msg.angular.x = ang_vel[0]
        twist_msg.angular.y = ang_vel[1]
        twist_msg.angular.z = ang_vel[2]
        return twist_msg
    
    def transform_twist_to_ee(self, twist_cam, stamp):
        """将速度指令从相机坐标系转换到末端坐标系"""
        # 使用最新时间查询TF，避免时间戳过旧的问题
        cam_to_ee_tf = self.get_transform(self.ee_frame, self.camera_frame, rospy.Time(0))
        rotation = cam_to_ee_tf.transform.rotation
        R_cam_to_ee = tf3d.quaternions.quat2mat([
            rotation.w, rotation.x, rotation.y, rotation.z
        ])
        translation = cam_to_ee_tf.transform.translation
        t_cam_to_ee = np.array([translation.x, translation.y, translation.z])
        skew_sym = np.array([
            [0, -t_cam_to_ee[2], t_cam_to_ee[1]],
            [t_cam_to_ee[2], 0, -t_cam_to_ee[0]],
            [-t_cam_to_ee[1], t_cam_to_ee[0], 0]
        ])
        skew_R = skew_sym @ R_cam_to_ee
        adjoint = np.zeros((6, 6))
        adjoint[:3, :3] = R_cam_to_ee
        adjoint[:3, 3:] = skew_R
        adjoint[3:, 3:] = R_cam_to_ee
        twist_cam_vec = np.array([
            twist_cam.linear.x, twist_cam.linear.y, twist_cam.linear.z,
            twist_cam.angular.x, twist_cam.angular.y, twist_cam.angular.z
        ])
        twist_ee_vec = adjoint @ twist_cam_vec
        twist_ee = Twist()
        twist_ee.linear.x = twist_ee_vec[0]
        twist_ee.linear.y = twist_ee_vec[1]
        twist_ee.linear.z = twist_ee_vec[2]
        twist_ee.angular.x = twist_ee_vec[3]
        twist_ee.angular.y = twist_ee_vec[4]
        twist_ee.angular.z = twist_ee_vec[5]
        return twist_ee
    
    def transform_twist_to_base(self, twist_ee, stamp):
        """将速度指令从末端坐标系转换到基座坐标系"""
        # 使用最新时间查询TF，避免时间戳过旧的问题
        ee_to_base_tf = self.get_transform(self.base_frame, self.ee_frame, rospy.Time(0))
        rotation = ee_to_base_tf.transform.rotation
        R_ee_to_base = tf3d.quaternions.quat2mat([
            rotation.w, rotation.x, rotation.y, rotation.z
        ])
        twist_ee_vec = np.array([
            twist_ee.linear.x, twist_ee.linear.y, twist_ee.linear.z,
            twist_ee.angular.x, twist_ee.angular.y, twist_ee.angular.z
        ])
        twist_base_vec = np.zeros(6)
        twist_base_vec[:3] = R_ee_to_base @ twist_ee_vec[:3]
        twist_base_vec[3:] = R_ee_to_base @ twist_ee_vec[3:]
        twist_base = Twist()
        twist_base.linear.x = twist_base_vec[0]
        twist_base.linear.y = twist_base_vec[1]
        twist_base.linear.z = twist_base_vec[2]
        twist_base.angular.x = twist_base_vec[3]
        twist_base.angular.y = twist_base_vec[4]
        twist_base.angular.z = twist_base_vec[5]
        return twist_base
    
    def get_transform(self, target_frame, source_frame, time):
        """获取坐标系间的变换"""
        return self.tf_buffer.lookup_transform(
            target_frame, source_frame, time, rospy.Duration(0.1))
    
    def publish_zero_velocity(self):
        """发布零速度指令"""
        # 发布零关节速度
        zero_joint_vel = Float64MultiArray()
        if self.joint_names is not None:
            zero_joint_vel.data = [0.0] * len(self.joint_names)
        else:
            zero_joint_vel.data = [0.0] * 6  # 默认6个关节
        self.joint_vel_pub.publish(zero_joint_vel)
        
        # 同时发布零Twist（用于简单控制模式）
        zero_twist = Twist()
        rospy.logdebug("发布零速度指令")
        self.cmd_pub.publish(zero_twist)
    
    def _print_joint_velocities(self, joint_velocities):
        """打印关节速度信息"""
        if self.joint_names is not None and len(self.joint_names) == len(joint_velocities):
            vel_str = ", ".join([f"{name}: {vel:.3f}" for name, vel in zip(self.joint_names, joint_velocities)])
            rospy.loginfo_throttle(0.5, f"关节速度 (rad/s): [{vel_str}]")
        else:
            vel_str = ", ".join([f"{vel:.3f}" for vel in joint_velocities])
            rospy.loginfo_throttle(0.5, f"关节速度 (rad/s): [{vel_str}]")
        
        # 打印速度幅值
        vel_magnitude = np.linalg.norm(joint_velocities)
        rospy.loginfo_throttle(0.5, f"关节速度幅值: {vel_magnitude:.3f} rad/s")
    
    def _print_twist_velocity(self, twist, source="控制"):
        """打印Twist速度信息"""
        lin_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        ang_vel = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        
        lin_mag = np.linalg.norm(lin_vel)
        ang_mag = np.linalg.norm(ang_vel)
        
        rospy.loginfo_throttle(0.5, 
            f"[{source}] 线速度: [{twist.linear.x:.4f}, {twist.linear.y:.4f}, {twist.linear.z:.4f}] m/s, "
            f"幅值: {lin_mag:.4f} m/s")
        rospy.loginfo_throttle(0.5,
            f"[{source}] 角速度: [{twist.angular.x:.4f}, {twist.angular.y:.4f}, {twist.angular.z:.4f}] rad/s, "
            f"幅值: {ang_mag:.4f} rad/s")
    
    def _check_controller_status(self, event):
        """检查控制器状态"""
        try:
            from controller_manager_msgs.srv import ListControllers
            list_controllers = rospy.ServiceProxy('/ur10e_robot/controller_manager/list_controllers', ListControllers)
            response = list_controllers()
            
            vel_controller_found = False
            for controller in response.controller:
                if controller.name == 'joint_group_vel_controller':
                    vel_controller_found = True
                    if controller.state == 'running':
                        rospy.loginfo("✓ joint_group_vel_controller 正在运行")
                    else:
                        rospy.logwarn(f"⚠ joint_group_vel_controller 状态: {controller.state} (期望: running)")
                        rospy.logwarn("控制器可能正在启动中，或请手动运行:")
                        rospy.logwarn("  rosrun ur10e_vs_node start_vel_controller.py")
                        rospy.logwarn("或使用rqt手动启动控制器")
                        break
            
            if not vel_controller_found:
                rospy.logerr("✗ 未找到 joint_group_vel_controller")
                
        except Exception as e:
            rospy.logwarn(f"无法检查控制器状态: {e}")
    
    def _check_aruco_timeout(self, event):
        """检查ArUco检测超时，如果超时则停止运动"""
        try:
            current_time = rospy.Time.now()
            
            # 如果还没有收到过有效的ArUco消息，不执行任何操作
            if self.last_aruco_time is None:
                return
            
            # 计算时间差
            time_since_last = (current_time - self.last_aruco_time).to_sec()
            
            # 如果超时，停止运动
            if time_since_last > self.aruco_timeout:
                rospy.logwarn_throttle(1.0, 
                    f"⚠ ArUco检测超时 ({time_since_last:.2f}秒 > {self.aruco_timeout}秒)，停止运动")
                self.publish_zero_velocity()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"ArUco超时检测出错: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = VisualServoMPCController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("控制器已关闭")

