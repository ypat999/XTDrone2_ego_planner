import os
import platform
from sys import prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 检查主机名，设置默认namespace
hostname = platform.node()
if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
    default_namespace = '/x500_depth_0/'
    use_sim_time = True
    use_sim_time_str = 'true'
    super_lio_launch_file = 'gazebo_mid360_drone.py'
else:
    default_namespace = '/'
    use_sim_time = False
    use_sim_time_str = 'false'  
    super_lio_launch_file = 'Livox_mid360_drone.py'

build_map_mode = os.environ.get('BUILD_MAP', '').lower() == 'true'


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', 
                    # default_value='aruco', 
                    # default_value='tugbot_warehouse',
                    default_value='ego',

                    description='Name of the world to launch (without .sdf)')
    model_name_arg = DeclareLaunchArgument('model_name', default_value='gz_x500_depth', description='Name of the model to spawn')
    id_arg = DeclareLaunchArgument('id', default_value='0', description='ID of the model to spawn')
    name_space_arg = DeclareLaunchArgument('namespace', default_value=default_namespace, description='ROS namespace for the model')


    #######################
    # Rosbridge Server   #
    #######################
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090,
            'address': '0.0.0.0',
        }],
        output='screen',
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    )

    #######################
    # Web Server         #
    #######################
    web_server = ExecuteProcess(
        cmd=['taskset', '-c', '0,1,2,3', 'python3', '-m', 'http.server', '8084'],
        output='screen',
        name='web_server',
        shell=False,
        cwd=PathJoinSubstitution([
            FindPackageShare('xtd2_launch'),
            'web'
        ])
    )

    #######################
    # Web PointCloud Bridge #
    #######################
    web_pointcloud_bridge = Node(
        package='xtd2_launch',
        executable='web_pointcloud_bridge',
        name='web_pointcloud_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    )

    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
    ])


    # 启动Rosbridge和Web服务器
    ld.add_action(rosbridge_node)
    ld.add_action(rosapi_node)
    ld.add_action(TimerAction(period=2.0, actions=[web_server]))
    ld.add_action(TimerAction(period=3.0, actions=[web_pointcloud_bridge]))
    

    return ld
