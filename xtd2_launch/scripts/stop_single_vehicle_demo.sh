#!/bin/bash

# XTDrone2 单机演示停止脚本

# 查找并终止 ros2_single_vehicle_demo_launch 进程
pkill -f "ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py"

# 查找并终止相关的 ROS2 节点
pkill -f "ego_planner_node"
pkill -f "tf_publisher"
pkill -f "px4_launch"
pkill -f "bridge_launch"
pkill -f "multirotor_communication"

echo "XTDrone2 Single Vehicle Demo 已停止"
