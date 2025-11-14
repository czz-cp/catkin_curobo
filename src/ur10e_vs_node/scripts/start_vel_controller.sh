#!/bin/bash
# 启动关节速度控制器的脚本

echo "等待控制器管理器就绪..."
sleep 5

echo "启动 joint_group_vel_controller..."
rosservice call /ur10e_robot/controller_manager/switch_controller \
  "start_controllers: ['joint_group_vel_controller']
stop_controllers: []
strictness: 2
start_asap: false
timeout: 0.0"

echo "检查控制器状态..."
rosservice call /ur10e_robot/controller_manager/list_controllers | grep -A 5 "joint_group_vel_controller"

