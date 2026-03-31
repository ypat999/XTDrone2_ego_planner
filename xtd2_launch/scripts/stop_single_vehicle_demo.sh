#!/bin/bash

# XTDrone2 单机演示停止脚本

# 查找并终止 ros2_single_vehicle_demo_launch 进程
pkill -9 -f "ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py"

# 查找并终止相关的 ROS2 节点
pkill -9 -f "ego_planner_node"
pkill -9 -f "tf_publisher"
pkill -9 -f "px4_launch"
pkill -9 -f "bridge_launch"
pkill -9 -f "multirotor_communication"

echo "XTDrone2 Single Vehicle Demo 已停止"
