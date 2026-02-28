#!/bin/bash
echo "===== ROS2 统一导航启动脚本 ====="
WORKSPACE_DIR="/home/cat/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2"

# 设置导航模式
export MANUAL_BUILD_MAP=False
# export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
# echo "显示输出：$DISPLAY"
export PYTHONPATH="$PYTHONPATH:/home/cat/.local/lib/python3.10/site-packages"


# 检查是否已加载ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "加载ROS2环境..."
    source /opt/ros/humble/setup.bash

    if [ $? -ne 0 ]; then
        echo "错误: 无法加载ROS2环境"
        exit 1
    fi
fi

# 加载工作空间环境
echo "加载工作空间环境..."
source $WORKSPACE_DIR/install/setup.bash


# echo "启动fast_lio..."
# ros2 launch fast_lio mapping.launch.py
echo "启动fast_lio中,pcd保存在/home/cat/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/Super-LIO/src/super_lio/map/map.pcd"
ros2 launch super_lio Livox_mid360.py
