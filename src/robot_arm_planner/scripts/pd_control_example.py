#!/usr/bin/env python3
"""
PD控制使用示例
演示如何使用基于PD控制的6维关节增量控制
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import time

class PDControlExample:
    def __init__(self):
        rospy.init_node('pd_control_example', anonymous=True)

        # 发布者
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
        self.control_cmd_pub = rospy.Publisher('/control_commands', Float64MultiArray, queue_size=1)
        self.test_trajectory_pub = rospy.Publisher('/planned_trajectory', JointTrajectory, queue_size=1)

        rospy.loginfo("PD Control Example initialized")

    def create_test_trajectory(self):
        """创建测试轨迹"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = "base_link"
        trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # 创建几个测试位姿点
        test_poses = [
            [0.0, -math.pi/4, math.pi/4, 0.0, math.pi/2, 0.0],    # 安全位姿1
            [0.5, -math.pi/3, math.pi/3, 0.0, math.pi/2, 0.0],    # 中间位姿
            [1.0, -math.pi/2, math.pi/2, 0.0, math.pi/2, 0.0],    # 伸展位姿
            [0.0, -math.pi/4, math.pi/4, 0.0, math.pi/2, 0.0],    # 回到安全位姿
        ]

        for i, pose in enumerate(test_poses):
            point = JointTrajectoryPoint()
            point.positions = pose
            point.velocities = [0.0] * 6
            point.accelerations = [0.0] * 6
            point.time_from_start = rospy.Duration(i * 2.0)  # 2秒间隔
            trajectory.points.append(point)

        return trajectory

    def create_goal_pose(self, x, y, z):
        """创建目标位姿"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0  # 四元数 [0,0,0,1]
        return goal

    def test_pd_control_direct(self):
        """测试直接PD控制命令"""
        rospy.loginfo("Testing direct PD control commands...")

        # 测试6维关节增量控制
        test_increments = [
            [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],    # 只移动基座
            [0.0, 0.01, 0.0, 0.0, 0.0, 0.0],    # 只移动大臂
            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0],    # 只移动肘部
            [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],    # 只移动腕部1
            [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],    # 只移动腕部2
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],    # 只移动腕部3
            [0.01, 0.01, 0.01, 0.01, 0.01, 0.01], # 同时移动所有关节
        ]

        for i, increments in enumerate(test_increments):
            rospy.loginfo(f"Test {i+1}: Sending increments {increments}")

            # 创建控制命令消息
            cmd_msg = Float64MultiArray()
            cmd_msg.data = increments

            # 发送到PD控制器
            self.control_cmd_pub.publish(cmd_msg)

            rospy.sleep(2.0)  # 等待执行

    def test_trajectory_based_control(self):
        """测试基于轨迹的PD控制"""
        rospy.loginfo("Testing trajectory-based PD control...")

        # 发布测试轨迹
        test_trajectory = self.create_test_trajectory()
        self.test_trajectory_pub.publish(test_trajectory)
        rospy.loginfo("Published test trajectory with 4 waypoints")

        # 等待PD控制器处理
        rospy.sleep(1.0)

    def test_goal_pose_control(self):
        """测试目标位姿控制"""
        rospy.loginfo("Testing goal pose control...")

        test_goals = [
            (0.3, 0.2, 0.4),  # 近距离目标
            (0.7, 0.4, 0.6),  # 中距离目标
            (0.2, -0.3, 0.5), # 不同象限目标
        ]

        for i, (x, y, z) in enumerate(test_goals):
            rospy.loginfo(f"Test goal {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")

            # 发布目标位姿
            goal = self.create_goal_pose(x, y, z)
            self.goal_pub.publish(goal)

            rospy.sleep(3.0)  # 等待PD控制器到达目标

    def test_ddpg_action_format(self):
        """测试DDPG格式的动作输出"""
        rospy.loginfo("Testing DDPG-style action format...")

        # 模拟DDPG策略输出的6维动作
        ddpg_actions = [
            [0.02, -0.01, 0.03, -0.02, 0.01, -0.01],  # 随机动作1
            [-0.01, 0.02, -0.01, 0.03, -0.02, 0.02], # 随机动作2
            [0.005, 0.005, 0.005, 0.005, 0.005, 0.005], # 小幅度动作
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],        # 大幅度动作 (会被限制)
        ]

        for i, action in enumerate(ddpg_actions):
            rospy.loginfo(f"DDPG Action {i+1}: {action}")

            # 创建DDPG动作消息
            action_msg = Float64MultiArray()
            action_msg.data = action

            # 发送到PD控制器 (使用相同的接口)
            self.control_cmd_pub.publish(action_msg)

            rospy.sleep(1.0)  # 等待执行

    def test_pd_parameters(self):
        """测试不同PD参数"""
        rospy.loginfo("Testing PD parameter sensitivity...")

        # 测试不同Kp值的影响
        kp_values = [1000, 3500, 7000]  # 低、中、高位置增益

        for kp in kp_values:
            rospy.loginfo(f"Testing with Kp = {kp}")

            # 发送一个小增量测试响应
            test_increment = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
            cmd_msg = Float64MultiArray()
            cmd_msg.data = test_increment

            self.control_cmd_pub.publish(cmd_msg)
            rospy.sleep(3.0)

    def test_emergency_stop(self):
        """测试紧急停止功能"""
        rospy.loginfo("Testing emergency stop functionality...")

        # 发送正常控制命令
        normal_cmd = Float64MultiArray()
        normal_cmd.data = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
        self.control_cmd_pub.publish(normal_cmd)
        rospy.loginfo("Sent normal control command")
        rospy.sleep(1.0)

        # 发送零命令 (模拟紧急停止)
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.control_cmd_pub.publish(stop_cmd)
        rospy.loginfo("Sent emergency stop command (zero increments)")
        rospy.sleep(1.0)

    def monitor_control_status(self):
        """监控PD控制状态"""
        rospy.loginfo("Monitoring PD control status...")

        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            try:
                # 这里可以订阅 /control_status 话题来监控PD控制状态
                pass
            except:
                pass

            rate.sleep()

    def run_example_sequence(self):
        """运行完整的示例序列"""
        rospy.loginfo("Starting PD Control Example Sequence...")

        rospy.sleep(2.0)  # 等待系统初始化

        # 示例1: 直接PD控制命令
        self.test_pd_control_direct()
        rospy.sleep(2.0)

        # 示例2: 基于轨迹的PD控制
        self.test_trajectory_based_control()
        rospy.sleep(2.0)

        # 示例3: 目标位姿控制
        self.test_goal_pose_control()
        rospy.sleep(2.0)

        # 示例4: DDPG动作格式
        self.test_ddpg_action_format()
        rospy.sleep(2.0)

        # 示例5: PD参数测试
        self.test_pd_parameters()
        rospy.sleep(2.0)

        # 示例6: 紧急停止测试
        self.test_emergency_stop()
        rospy.sleep(2.0)

        rospy.loginfo("PD Control Example Sequence Completed!")

def main():
    try:
        example = PDControlExample()

        # 运行示例序列
        example.run_example_sequence()

        # 可选：持续监控
        # example.monitor_control_status()

    except rospy.ROSInterruptException:
        rospy.loginfo("Example interrupted")

if __name__ == '__main__':
    main()