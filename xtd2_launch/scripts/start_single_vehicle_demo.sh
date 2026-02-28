#!/bin/bash

# XTDrone2 单机演示启动脚本

# 进入工作空间
cd /home/cat/git/xtd2_ws

# Source ROS2 环境
source install/setup.bash

# 设置日志目录
LOG_DIR="/home/cat/git/xtd2_ws/logs"
mkdir -p $LOG_DIR

# 启动 ros2_single_vehicle_demo_launch
ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
