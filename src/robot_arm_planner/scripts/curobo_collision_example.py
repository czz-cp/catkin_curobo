#!/usr/bin/env python3
"""
cuRobo防碰撞机械臂路径规划使用��例
演示如何使用cuRobo内置的SDF/ESDF碰撞检测进行安全路径规划
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
import time

class CuRoboCollisionExample:
    def __init__(self):
        rospy.init_node('curobo_collision_example', anonymous=True)

        # 发布者
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
        self.test_trajectory_pub = rospy.Publisher('/planned_trajectory', JointTrajectory, queue_size=1)

        rospy.loginfo("CuRobo Collision Example initialized")

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
            [0.0, -math.pi/4, math.pi/4, 0.0, math.pi/2, 0.0],  # 安全位姿1
            [0.5, -math.pi/3, math.pi/3, 0.0, math.pi/2, 0.0],  # 中间位姿
            [1.0, -math.pi/2, math.pi/2, 0.0, math.pi/2, 0.0],  # 伸展位姿
            [0.0, -math.pi/4, math.pi/4, 0.0, math.pi/2, 0.0],  # 回到安全位姿
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

    def test_collision_free_planning(self):
        """测试无碰撞路径规划"""
        rospy.loginfo("Testing collision-free path planning...")

        # 创建目标位姿
        goal = self.create_goal_pose(0.5, 0.3, 0.5)
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal pose: x={goal.pose.position.x:.3f}, y={goal.pose.position.y:.3f}, z={goal.pose.position.z:.3f}")

        # 等待规划完成
        rospy.sleep(1.0)

        # 调用规划服务
        try:
            plan_service = rospy.ServiceProxy('/plan_collision_free_path', Trigger)
            response = plan_service()

            if response.success:
                rospy.loginfo("✅ Collision-free path planning successful!")
            else:
                rospy.logwarn(f"❌ Planning failed: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def test_trajectory_collision_check(self):
        """测试轨迹碰撞检查"""
        rospy.loginfo("Testing trajectory collision checking...")

        # 发布测试轨迹
        test_trajectory = self.create_test_trajectory()
        self.test_trajectory_pub.publish(test_trajectory)
        rospy.loginfo("Published test trajectory with 4 waypoints")

        # 等待处理完成
        rospy.sleep(1.0)

        # 调用碰撞检查服务
        try:
            check_service = rospy.ServiceProxy('/check_trajectory_collision', Trigger)
            response = check_service()

            if response.success:
                rospy.loginfo("✅ Trajectory is collision-free!")
            else:
                rospy.logwarn(f"❌ Trajectory has collisions: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run_example_sequence(self):
        """运行完整的示例序列"""
        rospy.loginfo("Starting cuRobo Collision Example...")

        rospy.sleep(2.0)  # 等待系统初始化

        # 示例1: 测试碰撞检测
        self.test_trajectory_collision_check()
        rospy.sleep(3.0)

        # 示例2: 测试路径规划
        self.test_collision_free_planning()
        rospy.sleep(3.0)

        # 示例3: 测试多个目标点
        test_goals = [
            (0.3, 0.2, 0.4),  # 近距离目标
            (0.7, 0.4, 0.6),  # 中距离目标
            (0.2, -0.3, 0.5), # 不同象限目标
        ]

        for i, (x, y, z) in enumerate(test_goals):
            rospy.loginfo(f"Testing goal {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
            goal = self.create_goal_pose(x, y, z)
            self.goal_pub.publish(goal)
            rospy.sleep(2.0)

            try:
                plan_service = rospy.ServiceProxy('/plan_collision_free_path', Trigger)
                response = plan_service()

                if response.success:
                    rospy.loginfo(f"✅ Goal {i+1} planning successful!")
                else:
                    rospy.logwarn(f"❌ Goal {i+1} planning failed: {response.message}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Goal {i+1} service call failed: {e}")

            rospy.sleep(2.0)

        rospy.loginfo("Example sequence completed!")

    def monitor_collision_status(self):
        """监控碰撞状态"""
        rospy.loginfo("Monitoring collision status...")

        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            try:
                # 获取碰撞状态 (如果有的话)
                # 这里可以订阅/collision_status话题
                pass
            except:
                pass

            rate.sleep()

def main():
    try:
        example = CuRoboCollisionExample()

        # 运行示例序列
        example.run_example_sequence()

        # 持续监控 (可选)
        # example.monitor_collision_status()

    except rospy.ROSInterruptException:
        rospy.loginfo("Example interrupted")

if __name__ == '__main__':
    main()