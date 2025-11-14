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
        
        # ArUco检测状态
        self.last_aruco_time = None
        self.aruco_timeout = 1.0  # 1秒内没有检测到ArUco就停止
        self.aruco_detected = False
        
        # 控制参数
        self.control_rate = 20  # Hz (机械臂需要更高频率)
        self.position_tolerance = 0.005  # 5mm
        self.orientation_tolerance = 0.02  # ~1度
        
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
        self.status_pub = rospy.Publisher("/controller_status", rospy.String, queue_size=1)
        
        # 控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), 
                                        self.control_loop)
        
        # 状态监控定时器
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)
        
        rospy.loginfo("机械臂MPC视觉伺服控制器已启动")
        rospy.loginfo(f"期望位姿: 位置={self.desired_position_in_cam}, 欧拉角={np.degrees(self.desired_euler_in_cam)}°")
        
    def aruco_callback(self, msg):
        """处理ArUco检测结果"""
        try:
            if msg is None:
                rospy.logwarn("接收到空的ArUco位姿")
                self.aruco_detected = False
                return
                
            # 更新检测时间
            self.last_aruco_time = rospy.Time.now()
            self.aruco_detected = True
            
            # 更新当前机器人位姿
            self.update_robot_pose(msg.header.stamp)
            
            # 计算目标在基座坐标系下的位姿
            self.update_target_pose(msg)
            
            # 发布目标位姿用于可视化
            if self.target_pose_base is not None:
                self.target_pub.publish(self.target_pose_base)
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"ArUco回调错误: {e}")
            self.aruco_detected = False
    
    def check_aruco_timeout(self):
        """检查ArUco检测是否超时"""
        if self.last_aruco_time is None:
            return True  # 从未检测到ArUco
            
        time_since_last_detection = (rospy.Time.now() - self.last_aruco_time).to_sec()
        return time_since_last_detection > self.aruco_timeout
    
    def update_target_pose(self, aruco_pose):
        """更新目标位姿"""
        try:
            # 计算相机坐标系中的目标位姿
            target_in_cam = self.compute_target_pose_in_cam()
            
            # 将目标位姿变换到基座坐标系
            self.target_pose_base = self.transform_pose_to_base(target_in_cam, aruco_pose.header.stamp)
            
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
        # 检查ArUco检测状态
        if self.check_aruco_timeout():
            if self.has_target:  # 只在状态改变时记录
                rospy.logwarn("ArUco检测超时，停止运动")
                self.has_target = False
            self.publish_zero_velocity()
            return
            
        if not self.has_target or self.current_pose_base is None:
            return
            
        try:
            # 检查是否已经到达目标
            if self.check_target_reached():
                rospy.loginfo_throttle(2.0, "已到达目标位置")
                self.publish_zero_velocity()
                return
            
            # 更新MPC当前状态
            self.mpc.update_current_state(self.current_pose_base)
            
            # 求解MPC
            if self.mpc.solve():
                # 获取控制指令 (Twist消息)
                twist_cmd = self.mpc.get_control_twist()
                
                # 发布控制指令
                self.cmd_pub.publish(twist_cmd)
                
                # 发布预测轨迹用于可视化
                self.publish_mpc_trajectory()
                
                # 发布当前位姿用于可视化
                self.current_pose_pub.publish(self.current_pose_base)
                
                rospy.loginfo_throttle(1.0, 
                    f"MPC控制: lin=({twist_cmd.linear.x:.3f}, {twist_cmd.linear.y:.3f}, {twist_cmd.linear.z:.3f}) "
                    f"ang=({np.degrees(twist_cmd.angular.x):.1f}, {np.degrees(twist_cmd.angular.y):.1f}, {np.degrees(twist_cmd.angular.z):.1f}) deg/s")
                    
            else:
                rospy.logwarn("MPC求解失败，发布零速度")
                self.publish_zero_velocity()
                
        except Exception as e:
            rospy.logwarn(f"控制循环错误: {e}")
            self.publish_zero_velocity()
    
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
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                stamp,
                rospy.Duration(0.1)
            )
            
            self.current_pose_base = PoseStamped()
            self.current_pose_base.header = transform.header
            self.current_pose_base.pose.position = transform.transform.translation
            self.current_pose_base.pose.orientation = transform.transform.rotation
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"更新机器人位姿失败: {e}")
            self.current_pose_base = None
    
    def transform_pose_to_base(self, pose, stamp):
        """将位姿变换到基座坐标系"""
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            pose.header.frame_id,
            stamp,
            rospy.Duration(0.1)
        )
        
        pose_base = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return pose_base
    
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
    
    def publish_status(self, event):
        """发布控制器状态"""
        status_msg = ""
        if self.check_aruco_timeout():
            status_msg = "NO_ARUCO: 视野中没有检测到ArUco码"
        elif self.has_target and self.check_target_reached():
            status_msg = "TARGET_REACHED: 已到达目标位置"
        elif self.has_target:
            status_msg = "TRACKING: 正在跟踪ArUco码"
        else:
            status_msg = "WAITING: 等待目标"
        
        self.status_pub.publish(status_msg)
    
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