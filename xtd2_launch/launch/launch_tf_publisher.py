#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    tf_publisher_node = Node(
        package='xtd2_launch',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        tf_publisher_node
    ])