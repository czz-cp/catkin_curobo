#!/bin/bash
# 诊断脚本：检查关节速度控制器是否正常运行

echo "=== 检查关节速度控制器状态 ==="

# 1. 检查控制器是否运行
echo ""
echo "1. 检查控制器列表："
rosservice call /ur10e_robot/controller_manager/list_controllers

# 2. 检查topic是否存在
echo ""
echo "2. 检查关节速度命令topic："
rostopic list | grep "joint_group_vel_controller/command"

# 3. 检查topic是否有订阅者
echo ""
echo "3. 检查topic信息："
rostopic info /ur10e_robot/joint_group_vel_controller/command

# 4. 检查是否有消息发布
echo ""
echo "4. 监听最近的消息（5秒）："
timeout 5 rostopic echo /ur10e_robot/joint_group_vel_controller/command

# 5. 检查关节状态
echo ""
echo "5. 检查关节状态topic："
rostopic info /joint_states

echo ""
echo "=== 诊断完成 ==="

