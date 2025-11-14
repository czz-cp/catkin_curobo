#!/usr/bin/env python3
"""
基于PD控制的关节增量执行器
输出6维关节角度增量，使用标准PD控制公式
action = [Δθ1, Δθ2, Δθ3, Δθ4, Δθ5, Δθ6]
对应UR10e的6个关节:
θ1: shoulder_pan_joint (基座旋转)
θ2: shoulder_lift_joint (大臂抬升)
θ3: elbow_joint (肘部弯曲)
θ4: wrist_1_joint (腕部旋转1)
θ5: wrist_2_joint (腕部旋转2)
θ6: wrist_3_joint (腕部旋转3)
"""

import rospy
import numpy as np
import threading
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Header
from geometry_msgs.msg import Twist
import actionlib

class PDControlExecutor:
    def __init__(self):
        rospy.init_node('pd_control_executor', anonymous=True)

        # PD控制参数
        self.Kp = 3500.0  # 位置增益 - 控制响应强度
        self.Kd = 100.0   # 阻尼增益 - 减少震荡

        # 控制状态
        self.current_joint_states = None
        self.target_joint_positions = None
        self.current_joint_velocities = None
        self.control_enabled = False
        self.emergency_stop = False

        # 关节名称 (UR10e)
        self.joint_names = [
            'shoulder_pan_joint',   # θ1: 基座旋转
            'shoulder_lift_joint',  # θ2: 大臂抬升
            'elbow_joint',          # θ3: 肘部弯曲
            'wrist_1_joint',        # θ4: 腕部旋转1
            'wrist_2_joint',        # θ5: 腕部旋转2
            'wrist_3_joint'         # θ6: 腕部旋转3
        ]

        # ROS接口
        self.setup_ros_interfaces()

        # 控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        rospy.loginfo("PD Control Executor initialized")
        rospy.loginfo(f"PD Parameters: Kp={self.Kp}, Kd={self.Kd}")

    def setup_ros_interfaces(self):
        """设置ROS接口"""

        # 订阅者
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber('/collision_free_trajectory', JointTrajectory, self.trajectory_callback, queue_size=1)
        rospy.Subscriber('/control_commands', Float64MultiArray, self.control_commands_callback, queue_size=1)
        rospy.Subscriber('/collision_status', Float64MultiArray, self.collision_status_callback, queue_size=1)

        # 发布者 - 发布6维关节增量
        self.joint_increment_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        self.control_status_pub = rospy.Publisher('/control_status', Float64MultiArray, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Float64MultiArray, queue_size=1)
        self.debug_increment_pub = rospy.Publisher('/debug_joint_increments', Float64MultiArray, queue_size=1)

        # 可视化发布者
        self.current_pos_pub = rospy.Publisher('/current_joint_positions', Float64MultiArray, queue_size=1)
        self.target_pos_pub = rospy.Publisher('/target_joint_positions', Float64MultiArray, queue_size=1)

    def joint_states_callback(self, msg):
        """关节状态回调"""
        # 过滤UR10e关节并提取数据
        if all(name in msg.name for name in self.joint_names):
            self.current_joint_states = msg

            # 提取关节位置和速度
            self.current_joint_positions = []
            self.current_joint_velocities = []

            for joint_name in self.joint_names:
                idx = msg.name.index(joint_name)
                self.current_joint_positions.append(msg.position[idx])
                self.current_joint_velocities.append(msg.velocity[idx])

            self.current_joint_positions = np.array(self.current_joint_positions)
            self.current_joint_velocities = np.array(self.current_joint_velocities)

    def trajectory_callback(self, msg):
        """轨迹回调 - 提取第一个轨迹点作为目标"""
        if len(msg.points) > 0:
            first_point = msg.points[0]
            self.target_joint_positions = np.array(first_point.positions)
            self.control_enabled = True
            rospy.loginfo(f"Received trajectory target: {self.target_joint_positions}")
        else:
            rospy.logwarn("Received empty trajectory")

    def control_commands_callback(self, msg):
        """直接控制命令回调 - 接收6维关节增量"""
        if len(msg.data) == 6:
            self.publish_joint_increments(np.array(msg.data))

    def collision_status_callback(self, msg):
        """碰撞状态回调"""
        if len(msg.data) >= 1 and msg.data[0] > 0:
            rospy.logwarn(f"Collision detected! Count: {msg.data[0]}")
            self.trigger_emergency_stop()

    def compute_pd_control(self):
        """
        计算PD控制输出
        标准PD控制公式:
        data.ctrl[i] = -Kp * (qpos_i - target_i) - Kd * (qvel_i - 0)

        其中:
        Kp = 3500 (位置增益) - 控制响应强度
        Kd = 100  (阻尼增益) - 减少震荡
        qpos_i - 当前关节角度
        target_i - 目标关节角度
        qvel_i - 当前关节角速度
        """
        if (self.current_joint_positions is None or
            self.target_joint_positions is None or
            self.current_joint_velocities is None):
            return np.zeros(6)

        # 计算位置误差
        position_error = self.current_joint_positions - self.target_joint_positions

        # 速度误差 (目标速度为0)
        velocity_error = self.current_joint_velocities - np.zeros(6)

        # PD控制公式
        # 注意：使用负号是因为控制器的符号约定
        control_output = -self.Kp * position_error - self.Kd * velocity_error

        return control_output

    def publish_joint_increments(self, increments):
        """发布6维关节增量到UR10e控制器"""
        try:
            # 创建ROS消息
            increment_msg = Float64MultiArray()
            increment_msg.data = increments.tolist()

            # 发布到joint_group_vel_controller
            self.joint_increment_pub.publish(increment_msg)

            # 调试输出
            debug_msg = Float64MultiArray()
            debug_msg.data = [float(self.control_enabled), float(self.emergency_stop)] + increments.tolist()
            self.debug_increment_pub.publish(debug_msg)

        except Exception as e:
            rospy.logerr(f"Failed to publish joint increments: {e}")

    def control_loop(self):
        """主控制循环 - 50Hz"""
        rate = rospy.Rate(50)  # 50Hz控制频率

        while not rospy.is_shutdown():
            try:
                if self.control_enabled and not self.emergency_stop:
                    # 计算PD控制输出 (6维关节增量)
                    control_increments = self.compute_pd_control()

                    # 限制关节增量范围 (安全考虑)
                    max_increment = 0.1  # 最大增量限制 (rad)
                    control_increments = np.clip(control_increments, -max_increment, max_increment)

                    # 发布关节增量
                    self.publish_joint_increments(control_increments)

                    # 发布状态
                    self.publish_control_status()

                # 可视化发布
                self.publish_visualization()

            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")
                self.trigger_emergency_stop()

            rate.sleep()

    def publish_control_status(self):
        """发布控制状态"""
        status = Float64MultiArray()
        status.data = [
            float(self.control_enabled),      # 控制启用状态
            float(self.emergency_stop),       # 紧急停止状态
            float(self.Kp),                  # 位置增益
            float(self.Kd),                  # 阻尼增益
        ]
        self.control_status_pub.publish(status)

    def publish_visualization(self):
        """发布可视化数据"""
        if self.current_joint_positions is not None:
            # 发布当前关节位置
            current_pos_msg = Float64MultiArray()
            current_pos_msg.data = self.current_joint_positions.tolist()
            self.current_pos_pub.publish(current_pos_msg)

        if self.target_joint_positions is not None:
            # 发布目标关节位置
            target_pos_msg = Float64MultiArray()
            target_pos_msg.data = self.target_joint_positions.tolist()
            self.target_pos_pub.publish(target_pos_msg)

    def trigger_emergency_stop(self):
        """触发紧急停止"""
        self.emergency_stop = True
        self.control_enabled = False
        rospy.logwarn("Emergency stop triggered!")

        # 发送紧急停止信号
        emergency_msg = Float64MultiArray()
        emergency_msg.data = [1.0, rospy.Time.now().to_sec()]
        self.emergency_stop_pub.publish(emergency_msg)

        # 发送零增量停止
        self.publish_joint_increments(np.zeros(6))

    def enable_control(self):
        """启用PD控制"""
        self.control_enabled = True
        self.emergency_stop = False
        rospy.loginfo("PD control enabled")

    def disable_control(self):
        """禁用PD控制"""
        self.control_enabled = False
        self.publish_joint_increments(np.zeros(6))
        rospy.loginfo("PD control disabled")

    def set_target_position(self, target_positions):
        """设置目标关节位置"""
        if len(target_positions) == 6:
            self.target_joint_positions = np.array(target_positions)
            self.control_enabled = True
            rospy.loginfo(f"Set target positions: {target_positions}")
        else:
            rospy.logerr("Target positions must be 6-dimensional")

    def run(self):
        """主循环"""
        rospy.loginfo("PD Control Executor running...")
        rospy.loginfo("Control frequency: 50Hz")
        rospy.loginfo("Output topic: /joint_group_vel_controller/command")

        rate = rospy.Rate(10)  # 10Hz主循环
        try:
            while not rospy.is_shutdown():
                # 发布状态
                self.publish_control_status()
                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down PD Control Executor")

class ControlCommandInterface:
    """控制命令接口 - 方便外部调用"""

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/control_commands', Float64MultiArray, queue_size=1)
        self.status_sub = rospy.Subscriber('/control_status', Float64MultiArray, self.status_callback)
        self.control_enabled = False

    def status_callback(self, msg):
        """状态回调"""
        if len(msg.data) >= 1:
            self.control_enabled = bool(msg.data[0])

    def send_joint_increments(self, increments):
        """发送6维关节增量控制命令

        Args:
            increments: [Δθ1, Δθ2, Δθ3, Δθ4, Δθ5, Δθ6]
        """
        if len(increments) == 6:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = increments
            self.cmd_pub.publish(cmd_msg)
            return True
        else:
            rospy.logerr("Joint increments must be 6-dimensional")
            return False

    def send_control_action(self, action):
        """发送控制动作 (与DDPG兼容的接口)

        Args:
            action: 6维控制向量
        """
        return self.send_joint_increments(action)

def main():
    try:
        # 创建PD控制执行器
        executor = PDControlExecutor()

        # 可选：创建命令接口
        cmd_interface = ControlCommandInterface()

        # 运行主循环
        executor.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()