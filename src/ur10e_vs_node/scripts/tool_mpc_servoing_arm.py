#!/usr/bin/env python3
# tool_mpc_servoing_arm.py
import rospy
import numpy as np
import transforms3d as tf3d
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import tf.transformations as tf_trans
from mpc_controller_arm import MPCArmController

class MPCArmVisualServoingController:
    def __init__(self):
        rospy.init_node('mpc_arm_visual_servoing_controller')
        
        # 坐标系配置
        self.base_frame = "base"
        self.ee_frame = "tool0_controller"
        self.camera_frame = "camera_link"
        self.aruco_frame = "aruco_marker"
        
        # MPC控制器
        self.mpc = MPCArmController()
        
        # 期望的ArUco在相机坐标系下的位姿
        self.desired_position_in_cam = np.array([0.0, 0.0, 0.4])  # 相机前方40cm
        self.desired_euler_in_cam = np.array([np.pi, 0.0, 0.0])   # 绕X轴旋转180度
        self.desired_quat_in_cam = tf3d.euler.euler2quat(
            self.desired_euler_in_cam[0],
            self.desired_euler_in_cam[1],
            self.desired_euler_in_cam[2],
            axes='sxyz'
        )
        
        # 目标位姿存储
        self.target_pose_base = None
        self.current_pose_base = None
        self.has_target = False
        
        # 控制参数 - 提高响应速度
        self.control_rate = 100  # 提高控制频率到50Hz
        self.position_tolerance = 0.005  # 5mm，更精确的容差
        self.orientation_tolerance = 0.02  # ~1度，更精确的容差
        
        # 添加控制状态跟踪
        self.last_control_time = rospy.Time.now()
        self.control_timeout = rospy.Duration(0.2)  # 200ms超时，更快响应
        self.consecutive_failures = 0
        
        # 添加响应性参数
        self.max_velocity_scale = 1.5  # 速度缩放因子
        self.acceleration_limit = 0.5  # 加速度限制
        
        # 添加直接控制模式
        self.use_direct_control = False  # 是否使用直接控制模式
        self.mpc_failure_count = 0
        self.mpc_failure_threshold = 5  # MPC连续失败5次后切换到直接控制
        
        # TF工具
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # 订阅者 & 发布者
        rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_callback)
        self.cmd_pub = rospy.Publisher("/ur10e_robot/twist_controller/command", 
                                      Twist, queue_size=1)
        self.mpc_trajectory_pub = rospy.Publisher("/mpc_trajectory", Path, queue_size=1)
        self.target_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)
        self.current_pose_pub = rospy.Publisher("/current_pose", PoseStamped, queue_size=1)
        
        # 控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), 
                                        self.control_loop)
        
        rospy.loginfo("机械臂MPC视觉伺服控制器已启动")
        rospy.loginfo(f"期望位姿: 位置={self.desired_position_in_cam}, 欧拉角={np.degrees(self.desired_euler_in_cam)}°")
        
    def aruco_callback(self, msg):
        """处理ArUco检测结果"""
        try:
            if msg is None:
                rospy.logwarn("接收到空的ArUco位姿")
                return
                
            # 更新当前机器人位姿
            self.update_robot_pose(msg.header.stamp)
            
            # 计算目标在基座坐标系下的位姿
            self.update_target_pose(msg)
            
            # 发布目标位姿用于可视化
            if self.target_pose_base is not None:
                self.target_pub.publish(self.target_pose_base)
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"ArUco回调错误: {e}")
    
    def update_target_pose(self, aruco_pose):
        """更新目标位姿"""
        try:
            # 计算相机坐标系中的目标位姿
            target_in_cam = self.compute_target_pose_in_cam()
            
            # 将目标位姿变换到基座坐标系
            self.target_pose_base = self.transform_pose_to_base(target_in_cam, aruco_pose.header.stamp)
            
            # 检查变换是否成功
            if self.target_pose_base is None:
                rospy.logwarn_throttle(1.0, "目标位姿变换失败")
                self.has_target = False
                return
            
            # 检查目标位姿是否合理
            target_pos = np.array([
                self.target_pose_base.pose.position.x,
                self.target_pose_base.pose.position.y,
                self.target_pose_base.pose.position.z
            ])
            
            # 检查目标是否在工作空间内
            if np.linalg.norm(target_pos) > 2.0:  # 距离基座超过2米
                rospy.logwarn_throttle(2.0, f"目标位姿超出工作空间: {np.linalg.norm(target_pos):.3f}m")
                self.has_target = False
                return
            
            # 设置MPC目标
            self.mpc.set_target_state(self.target_pose_base)
            self.has_target = True
            
            rospy.loginfo_throttle(2.0, 
                f"目标位姿更新: pos=({self.target_pose_base.pose.position.x:.3f}, "
                f"{self.target_pose_base.pose.position.y:.3f}, "
                f"{self.target_pose_base.pose.position.z:.3f})")
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"更新目标位姿失败: {e}")
            self.has_target = False
    
    def control_loop(self, event):
        """MPC控制循环"""
        current_time = rospy.Time.now()
        
        # 检查是否有有效的目标位姿和当前位姿
        if not self.has_target or self.current_pose_base is None:
            rospy.logdebug_throttle(2.0, "等待有效位姿数据...")
            return
        
        try:
            # 检查是否已经到达目标
            if self.check_target_reached():
                rospy.loginfo_throttle(2.0, "已到达目标位置")
                self.publish_zero_velocity()
                self.consecutive_failures = 0
                return
            
            # 更新MPC当前状态
            self.mpc.update_current_state(self.current_pose_base)
            
            # 选择控制模式
            if self.use_direct_control:
                # 直接控制模式
                twist_cmd = self._direct_control()
                if twist_cmd is not None:
                    self.cmd_pub.publish(twist_cmd)
                    self.last_control_time = current_time
                    self.consecutive_failures = 0
                    rospy.loginfo_throttle(1.0, "使用直接控制模式")
                else:
                    self.publish_zero_velocity()
                    self.consecutive_failures += 1
            else:
                # MPC控制模式
                if self.mpc.solve():
                    # 获取控制指令 (Twist消息)
                    twist_cmd = self.mpc.get_control_twist()
                    
                    # 检查控制指令是否合理
                    if self._check_control_command(twist_cmd):
                        # 应用速度缩放和响应性优化
                        twist_cmd = self._apply_velocity_scaling(twist_cmd)
                        
                        # 发布控制指令
                        self.cmd_pub.publish(twist_cmd)
                        rospy.logwarn_throttle(1.0, "控制指令已发布")
                        self.last_control_time = current_time
                        self.consecutive_failures = 0
                        self.mpc_failure_count = 0  # 重置失败计数
                        
                        # 发布预测轨迹用于可视化
                        self.publish_mpc_trajectory()
                        
                        # 发布当前位姿用于可视化
                        self.current_pose_pub.publish(self.current_pose_base)
                        
                        rospy.loginfo_throttle(1.0, 
                            f"MPC控制: lin=({twist_cmd.linear.x:.3f}, {twist_cmd.linear.y:.3f}, {twist_cmd.linear.z:.3f}) "
                            f"ang=({np.degrees(twist_cmd.angular.x):.1f}, {np.degrees(twist_cmd.angular.y):.1f}, {np.degrees(twist_cmd.angular.z):.1f}) deg/s")
                    else:
                        rospy.logwarn_throttle(1.0, "控制指令异常，发布零速度")
                        self.publish_zero_velocity()
                        self.consecutive_failures += 1
                        self.mpc_failure_count += 1
                        
                else:
                    rospy.logwarn_throttle(1.0, "MPC求解失败，发布零速度")
                    self.publish_zero_velocity()
                    self.consecutive_failures += 1
                    self.mpc_failure_count += 1
                    
                    # 检查是否需要切换到直接控制模式
                    if self.mpc_failure_count >= self.mpc_failure_threshold:
                        rospy.logwarn("MPC连续失败，切换到直接控制模式")
                        self.use_direct_control = True
                        self.mpc_failure_count = 0
                
        except Exception as e:
            rospy.logwarn(f"控制循环错误: {e}")
            self.publish_zero_velocity()
            self.consecutive_failures += 1
    
    def _check_control_command(self, twist_cmd):
        """检查控制指令是否合理"""
        try:
            # 检查线速度
            linear_vel = np.array([twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z])
            linear_magnitude = np.linalg.norm(linear_vel)
            
            # 检查角速度
            angular_vel = np.array([twist_cmd.angular.x, twist_cmd.angular.y, twist_cmd.angular.z])
            angular_magnitude = np.linalg.norm(angular_vel)
            
            # 检查是否包含NaN或无穷大值
            if (np.any(np.isnan(linear_vel)) or np.any(np.isinf(linear_vel)) or
                np.any(np.isnan(angular_vel)) or np.any(np.isinf(angular_vel))):
                rospy.logwarn("控制指令包含NaN或无穷大值")
                return False
            
            # 检查速度是否在合理范围内
            if linear_magnitude > 1.0:  # 线速度超过20cm/s
                rospy.logwarn(f"线速度过大: {linear_magnitude:.3f} m/s")
                return False
                
            if angular_magnitude > 10.0:  # 角速度超过1 rad/s
                rospy.logwarn(f"角速度过大: {angular_magnitude:.3f} rad/s")
                return False
            
            return True
            
        except Exception as e:
            rospy.logwarn(f"控制指令检查失败: {e}")
            return False
    
    def _apply_velocity_scaling(self, twist_cmd):
        """应用速度缩放以提高响应性"""
        try:
            # 计算当前误差
            if self.current_pose_base is not None and self.target_pose_base is not None:
                pos_error = np.array([
                    self.target_pose_base.pose.position.x - self.current_pose_base.pose.position.x,
                    self.target_pose_base.pose.position.y - self.current_pose_base.pose.position.y,
                    self.target_pose_base.pose.position.z - self.current_pose_base.pose.position.z
                ])
                
                error_magnitude = np.linalg.norm(pos_error)
                
                # 根据误差大小动态调整速度缩放
                if error_magnitude > 0.1:  # 误差大于10cm时使用最大速度
                    velocity_scale = self.max_velocity_scale
                elif error_magnitude > 0.05:  # 误差5-10cm时使用中等速度
                    velocity_scale = 1.0
                else:  # 误差小于5cm时使用较低速度
                    velocity_scale = 0.5
                
                # 应用速度缩放
                twist_cmd.linear.x *= velocity_scale
                twist_cmd.linear.y *= velocity_scale
                twist_cmd.linear.z *= velocity_scale
                twist_cmd.angular.x *= velocity_scale
                twist_cmd.angular.y *= velocity_scale
                twist_cmd.angular.z *= velocity_scale
                
                rospy.logdebug(f"速度缩放: scale={velocity_scale:.2f}, error={error_magnitude:.3f}m")
            
            return twist_cmd
            
        except Exception as e:
            rospy.logwarn(f"速度缩放失败: {e}")
            return twist_cmd
    
    def _direct_control(self):
        """直接控制模式 - 简单的比例控制"""
        try:
            if self.current_pose_base is None or self.target_pose_base is None:
                return None
            
            # 计算位置误差
            pos_error = np.array([
                self.target_pose_base.pose.position.x - self.current_pose_base.pose.position.x,
                self.target_pose_base.pose.position.y - self.current_pose_base.pose.position.y,
                self.target_pose_base.pose.position.z - self.current_pose_base.pose.position.z
            ])
            
            # 计算姿态误差
            current_quat = [
                self.current_pose_base.pose.orientation.x,
                self.current_pose_base.pose.orientation.y,
                self.current_pose_base.pose.orientation.z,
                self.current_pose_base.pose.orientation.w
            ]
            target_quat = [
                self.target_pose_base.pose.orientation.x,
                self.target_pose_base.pose.orientation.y,
                self.target_pose_base.pose.orientation.z,
                self.target_pose_base.pose.orientation.w
            ]
            
            # 简化的姿态控制（仅使用位置控制）
            twist_cmd = Twist()
            
            # 比例控制增益
            kp_linear = 2.0  # 位置比例增益
            kp_angular = 1.0  # 姿态比例增益
            
            # 位置控制
            twist_cmd.linear.x = kp_linear * pos_error[0]
            twist_cmd.linear.y = kp_linear * pos_error[1]
            twist_cmd.linear.z = kp_linear * pos_error[2]
            
            # 简单的姿态控制（基于四元数误差）
            quat_error = self._quaternion_error(target_quat, current_quat)
            twist_cmd.angular.x = kp_angular * quat_error[0]
            twist_cmd.angular.y = kp_angular * quat_error[1]
            twist_cmd.angular.z = kp_angular * quat_error[2]
            
            # 限制速度
            max_vel = 0.3
            twist_cmd.linear.x = np.clip(twist_cmd.linear.x, -max_vel, max_vel)
            twist_cmd.linear.y = np.clip(twist_cmd.linear.y, -max_vel, max_vel)
            twist_cmd.linear.z = np.clip(twist_cmd.linear.z, -max_vel, max_vel)
            twist_cmd.angular.x = np.clip(twist_cmd.angular.x, -max_vel, max_vel)
            twist_cmd.angular.y = np.clip(twist_cmd.angular.y, -max_vel, max_vel)
            twist_cmd.angular.z = np.clip(twist_cmd.angular.z, -max_vel, max_vel)
            
            return twist_cmd
            
        except Exception as e:
            rospy.logwarn(f"直接控制失败: {e}")
            return None
    
    def _quaternion_error(self, target_quat, current_quat):
        """计算四元数误差"""
        try:
            # 简化的四元数误差计算
            error = np.array([
                target_quat[0] - current_quat[0],
                target_quat[1] - current_quat[1],
                target_quat[2] - current_quat[2]
            ])
            return error
        except Exception as e:
            rospy.logwarn(f"四元数误差计算失败: {e}")
            return np.zeros(3)
    
    def compute_target_pose_in_cam(self):
        """计算相机坐标系中的目标位姿"""
        target = PoseStamped()
        target.header.frame_id = self.camera_frame
        target.pose.position.x = self.desired_position_in_cam[0]
        target.pose.position.y = self.desired_position_in_cam[1]
        target.pose.position.z = self.desired_position_in_cam[2]
        
        target.pose.orientation.x = self.desired_quat_in_cam[1]
        target.pose.orientation.y = self.desired_quat_in_cam[2]
        target.pose.orientation.z = self.desired_quat_in_cam[3]
        target.pose.orientation.w = self.desired_quat_in_cam[0]
        
        return target
    
    def update_robot_pose(self, stamp):
        """更新机器人末端当前位置"""
        try:
            # 使用更长的超时时间，并尝试获取最新的变换
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rospy.Time(0),  # 获取最新的变换
                rospy.Duration(0.2)  # 增加超时时间
            )
            
            # 检查变换的时间戳是否太旧
            transform_age = rospy.Time.now() - transform.header.stamp
            if transform_age > rospy.Duration(0.5):  # 如果变换超过500ms，认为太旧
                rospy.logwarn_throttle(2.0, f"TF变换太旧: {transform_age.to_sec():.3f}s")
                self.current_pose_base = None
                return
            
            self.current_pose_base = PoseStamped()
            self.current_pose_base.header = transform.header
            self.current_pose_base.pose.position = transform.transform.translation
            self.current_pose_base.pose.orientation = transform.transform.rotation
            
            rospy.logdebug(f"机器人位姿更新: pos=({self.current_pose_base.pose.position.x:.3f}, "
                          f"{self.current_pose_base.pose.position.y:.3f}, "
                          f"{self.current_pose_base.pose.position.z:.3f})")
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"更新机器人位姿失败: {e}")
            self.current_pose_base = None
    
    def transform_pose_to_base(self, pose, stamp):
        """将位姿变换到基座坐标系"""
        try:
            # 使用更长的超时时间
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose.header.frame_id,
                rospy.Time(0),  # 获取最新的变换
                rospy.Duration(0.2)
            )
            
            # 检查变换的时间戳
            transform_age = rospy.Time.now() - transform.header.stamp
            if transform_age > rospy.Duration(0.5):
                rospy.logwarn_throttle(2.0, f"TF变换太旧: {transform_age.to_sec():.3f}s")
                return None
            
            pose_base = tf2_geometry_msgs.do_transform_pose(pose, transform)
            
            rospy.logdebug(f"位姿变换成功: {pose.header.frame_id} -> {self.base_frame}")
            return pose_base
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"位姿变换失败: {e}")
            return None
    
    def check_target_reached(self):
        """检查是否到达目标"""
        if self.current_pose_base is None or self.target_pose_base is None:
            return False
            
        # 位置误差
        pos_error = np.array([
            self.target_pose_base.pose.position.x - self.current_pose_base.pose.position.x,
            self.target_pose_base.pose.position.y - self.current_pose_base.pose.position.y,
            self.target_pose_base.pose.position.z - self.current_pose_base.pose.position.z
        ])
        
        # 姿态误差
        current_quat = [
            self.current_pose_base.pose.orientation.x,
            self.current_pose_base.pose.orientation.y,
            self.current_pose_base.pose.orientation.z,
            self.current_pose_base.pose.orientation.w
        ]
        target_quat = [
            self.target_pose_base.pose.orientation.x,
            self.target_pose_base.pose.orientation.y,
            self.target_pose_base.pose.orientation.z,
            self.target_pose_base.pose.orientation.w
        ]
        
        # 计算四元数夹角
        dot_product = np.abs(np.dot(current_quat, target_quat))
        orientation_error = 2 * np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        pos_error_magnitude = np.linalg.norm(pos_error)
        
        reached = (pos_error_magnitude < self.position_tolerance and 
                  orientation_error < self.orientation_tolerance)
        
        if reached:
            rospy.loginfo_once("已到达目标位置和姿态")
            
        return reached
    
    def publish_mpc_trajectory(self):
        """发布MPC预测轨迹"""
        predicted_poses = self.mpc.get_predicted_poses()
        if not predicted_poses:
            return
            
        path = Path()
        path.header.frame_id = self.base_frame
        path.header.stamp = rospy.Time.now()
        path.poses = predicted_poses
        
        self.mpc_trajectory_pub.publish(path)
    
    def publish_zero_velocity(self):
        """发布零速度指令"""
        zero_twist = Twist()
        self.cmd_pub.publish(zero_twist)
    
    def run(self):
        """运行控制器"""
        rospy.loginfo("机械臂MPC视觉伺服控制器运行中...")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MPCArmVisualServoingController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("控制器已关闭")
        controller.publish_zero_velocity()