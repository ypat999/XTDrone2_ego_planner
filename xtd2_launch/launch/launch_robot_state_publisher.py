#!/usr/bin/env python3

import platform
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# 检查主机名，设置默认use_sim_time
hostname = platform.node()
if hostname == 'ywj-B250-D3A':
    default_use_sim_time = True
else:
    default_use_sim_time = False

def generate_launch_description():
    
    # 获取URDF文件路径
    urdf_file_path = os.path.join(
        get_package_share_directory('xtd2_launch'),
        'urdf',
        'x500_depth.urdf'
    )
    
    # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='x500_depth_0',
        description='ROS namespace for the robot'
    )
    
    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file_path, 'r').read(),
            'use_sim_time': default_use_sim_time,
            'frame_prefix': LaunchConfiguration('namespace') + '/'
        }]
    )
    
    # 静态TF发布器 - 发布map到odom的变换
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Joint State Publisher GUI（可选，用于调试）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        static_transform_publisher,
        joint_state_publisher_gui_node
    ])