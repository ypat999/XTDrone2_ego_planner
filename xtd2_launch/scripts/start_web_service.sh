#!/bin/bash

# XTDrone2 单机演示启动脚本

# 进入工作空间
cd /home/cat/git/xtd2_ws

# Source ROS2 环境
source install/setup.bash

# 启动 web_service
ros2 launch xtd2_launch web_service.py
