#!/usr/bin/env python3
"""
启动关节速度控制器的脚本
"""
import rospy
import sys
from controller_manager_msgs.srv import SwitchController, ListControllers

def start_vel_controller():
    rospy.init_node('start_vel_controller', anonymous=True)
    
    # 等待服务可用
    rospy.loginfo("等待控制器管理器服务...")
    try:
        rospy.wait_for_service('/ur10e_robot/controller_manager/switch_controller', timeout=15.0)
        rospy.wait_for_service('/ur10e_robot/controller_manager/list_controllers', timeout=15.0)
    except rospy.ROSException as e:
        rospy.logerr(f"等待服务超时: {e}")
        return 1
    
    try:
        list_controllers = rospy.ServiceProxy(
            '/ur10e_robot/controller_manager/list_controllers',
            ListControllers
        )
        switch_controller = rospy.ServiceProxy(
            '/ur10e_robot/controller_manager/switch_controller', 
            SwitchController
        )
        
        # 先检查当前状态
        rospy.loginfo("检查控制器状态...")
        list_response = list_controllers()
        
        vel_controller_state = None
        pos_controller_state = None
        
        for controller in list_response.controller:
            if controller.name == 'joint_group_vel_controller':
                vel_controller_state = controller.state
                rospy.loginfo(f"joint_group_vel_controller 当前状态: {vel_controller_state}")
            if controller.name == 'scaled_pos_joint_traj_controller':
                pos_controller_state = controller.state
                rospy.loginfo(f"scaled_pos_joint_traj_controller 当前状态: {pos_controller_state}")
        
        # 如果速度控制器已经在运行，直接返回成功
        if vel_controller_state == 'running':
            rospy.loginfo("✓ joint_group_vel_controller 已经在运行")
            return 0
        
        # 如果速度控制器不存在，报错
        if vel_controller_state is None:
            rospy.logerr("✗ 未找到 joint_group_vel_controller")
            return 1
        
        # 尝试启动速度控制器
        # 注意：如果位置控制器正在运行，可能需要先停止它（硬件接口冲突）
        stop_controllers = []
        if pos_controller_state == 'running':
            rospy.logwarn("检测到位置控制器正在运行，将先停止它以避免硬件接口冲突")
            stop_controllers = ['scaled_pos_joint_traj_controller']
        
        rospy.loginfo("启动关节速度控制器...")
        response = switch_controller(
            start_controllers=['joint_group_vel_controller'],
            stop_controllers=stop_controllers,
            strictness=2,  # 2 = BEST_EFFORT (允许部分失败)
            start_asap=False,
            timeout=0.0
        )
        
        if response.ok:
            rospy.loginfo("✓ 成功启动 joint_group_vel_controller")
            # 验证一下
            rospy.sleep(1.0)
            list_response = list_controllers()
            for controller in list_response.controller:
                if controller.name == 'joint_group_vel_controller':
                    if controller.state == 'running':
                        rospy.loginfo("✓ 验证成功: joint_group_vel_controller 正在运行")
                    else:
                        rospy.logwarn(f"⚠ 警告: joint_group_vel_controller 状态为 {controller.state}")
                    break
            return 0
        else:
            rospy.logerr(f"✗ 启动失败")
            rospy.logerr(f"错误信息: {response}")
            # 尝试不停止位置控制器
            if stop_controllers:
                rospy.loginfo("尝试不停止位置控制器，直接启动速度控制器...")
                response2 = switch_controller(
                    start_controllers=['joint_group_vel_controller'],
                    stop_controllers=[],
                    strictness=2,
                    start_asap=False,
                    timeout=0.0
                )
                if response2.ok:
                    rospy.loginfo("✓ 成功启动（不停止位置控制器）")
                    return 0
            return 1
            
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return 1
    except Exception as e:
        rospy.logerr(f"错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        return 1

if __name__ == '__main__':
    try:
        sys.exit(start_vel_controller())
    except rospy.ROSInterruptException:
        rospy.loginfo("已中断")
        sys.exit(1)

