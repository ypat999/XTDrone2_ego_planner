#!/usr/bin/env python3

import platform
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# 检查主机名，设置默认namespace
hostname = platform.node()
if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
    default_namespace = '/x500_depth_0/'
    default_use_sim_time = True
else:
    default_namespace = '/'
    default_use_sim_time = False

def generate_launch_description():
    
    # 获取URDF文件路径
    urdf_file_path = os.path.join(
        get_package_share_directory('xtd2_launch'),
        'urdf',
        'x500_depth.urdf'
    )
    
    # 获取RViz配置文件路径
    rviz_config_path = os.path.join(
        get_package_share_directory('xtd2_launch'),
        'rviz',
        'x500_depth.rviz'
    )
    
    # 声明参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
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
            'frame_prefix': LaunchConfiguration('namespace').strip('/') + '/'
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
    
    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    # 启动原始模拟系统
    original_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xtd2_launch"),
                "launch",
                "ros2_single_vehicle_demo_launch.py"
            ])
        ])
    )
    
    return LaunchDescription([
        namespace_arg,
        static_transform_publisher,
        TimerAction(period=5.0, actions=[robot_state_publisher_node]),
        TimerAction(period=10.0, actions=[rviz_node]),
        TimerAction(period=2.0, actions=[original_simulation])
    ])