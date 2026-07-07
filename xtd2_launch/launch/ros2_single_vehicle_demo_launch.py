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
                    default_value='tugbot_warehouse',
                    # default_value='ego',

                    description='Name of the world to launch (without .sdf)')
    model_name_arg = DeclareLaunchArgument('model_name', default_value='gz_x500_depth', description='Name of the model to spawn')
    id_arg = DeclareLaunchArgument('id', default_value='0', description='ID of the model to spawn')
    name_space_arg = DeclareLaunchArgument('namespace', default_value=default_namespace, description='ROS namespace for the model')

    
    #####################
    # Gazebo Simulation #
    #####################
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xtd2_launch"),
                "launch",
                "gz_launch.py"
            ])
        ]),
        launch_arguments={
            "world": LaunchConfiguration('world_name'),
        }.items()
    )

    ##################
    # XRCE-DDS Agent #
    ##################
    xrce_dds_process = ExecuteProcess(
        cmd=["taskset", "-c", "0,1,2,3", "MicroXRCEAgent udp4 -p 8888"],
        output='screen',
        name='microxrceagent',
        shell=True
    )

    ###################################################################
    # Spawn vehicles, including Model, PX4 SITL and ROS-Gazebo bridge #
    ###################################################################
    spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('xtd2_launch'),
                    'launch',
                    'xtd2_vehicle_spawn_launch.py'
                ])
            ]),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'model': LaunchConfiguration('model_name'),
                'id': LaunchConfiguration('id'),
                'namespace': LaunchConfiguration('namespace'),
            }.items()
        )
    
    # XTDrone2 Communication
    xtd2_communication = Node(
        package='xtd2_communication',
        executable='multirotor_communication',
        output='screen',
        emulate_tty=True,
        shell=True,
        arguments=[
            "--model", LaunchConfiguration('model_name'),
            "--id", LaunchConfiguration('id'),
            "--allowarm", "true",
            "--namespace", LaunchConfiguration('namespace'),
            "--debug", "true",
            "--require-pcl-pose", "true" if not use_sim_time else "false",
        ],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 
    )

    ####################
    # TF Publisher Node #
    ####################
    tf_publisher = ExecuteProcess(
        cmd=["ros2", "run", "xtd2_launch", "tf_publisher"],
        output='screen',
        name='tf_publisher',
        shell=False,
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    )

    ##############################
    # Super LIO (真实飞机环境下启动)
    ##############################
    super_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('super_lio'),
                'launch',
                super_lio_launch_file
            ])
        ]),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': use_sim_time_str,
            'dynamic_removal': 'true' if build_map_mode else 'false',

        }.items()
    )

    ##############################
    # Lidar Localization
    ##############################
    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_localization_ros2'),
                'launch',
                'lidar_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': use_sim_time_str,
        }.items()
    )
    
    #######################
    ego_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xtd2_launch'),
                'launch',
                'ego_planner_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'drone_id': LaunchConfiguration('id'),
            'use_sim_time': use_sim_time_str,
        }.items()
    )

    #######################
    # RViz Visualization #
    #######################
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('xtd2_launch'),
            'rviz',
            'x500_depth.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # #######################
    # # Rosbridge Server   #
    # #######################
    # rosbridge_node = Node(
    #     package='rosbridge_server',
    #     executable='rosbridge_websocket',
    #     name='rosbridge_websocket',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'port': 9090,
    #         'address': '0.0.0.0',
    #     }],
    #     output='screen',
    #     prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    # )

    # rosapi_node = Node(
    #     package='rosapi',
    #     executable='rosapi_node',
    #     name='rosapi_node',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    # )

    # #######################
    # # Web Server         #
    # #######################
    # web_server = ExecuteProcess(
    #     cmd=['taskset', '-c', '0,1,2,3', 'python3', '-m', 'http.server', '8084'],
    #     output='screen',
    #     name='web_server',
    #     shell=False,
    #     cwd=PathJoinSubstitution([
    #         FindPackageShare('xtd2_launch'),
    #         'web'
    #     ])
    # )

    # #######################
    # # Web PointCloud Bridge #
    # #######################
    # web_pointcloud_bridge = Node(
    #     package='xtd2_launch',
    #     executable='web_pointcloud_bridge',
    #     name='web_pointcloud_bridge',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU 
    # )

    # 根据主机名决定启动哪些组件
    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
    ])

    px4_odom_topic = default_namespace + 'fmu/out/vehicle_odometry'
    
    # 等待px4_odom_topic发布
    wait_for_px4_odom = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'''
echo "等待话题发布: {px4_odom_topic}"
echo "最大等待时间: 300秒"
echo "检查间隔: 1秒"
echo "发布后等待时间: 5秒"

start_time=$(date +%s)
timeout=300
check_interval=1
post_wait_delay=5

while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
    if ros2 topic list | grep -q "{px4_odom_topic}"; then
        echo "话题 {px4_odom_topic} 已发布！"
        echo "等待 $post_wait_delay 秒后退出..."
        sleep $post_wait_delay
        exit 0
    else
        elapsed=$(($(date +%s) - start_time))
        echo "等待中... 已等待 $elapsed 秒"
    fi
    sleep $check_interval
done

echo "错误：等待超时！话题 {px4_odom_topic} 在 $timeout 秒内未发布"
exit 1
'''
        ],
        output='screen',
        name='wait_for_px4_odom',
        shell=False
    )

    # 当主机为ywj-B250-D3A或DESKTOP-ypat时，启动整套px4模拟
    if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
        
        ld.add_action(world_launch)  # 启动Gazebo模拟环境
        ld.add_action(xrce_dds_process)  # 启动XRCE-DDS Agent
        ld.add_action(TimerAction(period=15.0, actions=[spawn]))  # 启动模型、PX4 SITL和ROS-Gazebo桥接
        ld.add_action(TimerAction(period=10.0, actions=[rviz_node]))  # 启动RViz可视化
    
    
    ld.add_action(wait_for_px4_odom)  # 等待px4_odom_topic发布
    
    # # 启动Rosbridge和Web服务器
    # ld.add_action(rosbridge_node)
    # ld.add_action(rosapi_node)
    # ld.add_action(TimerAction(period=2.0, actions=[web_server]))
    # ld.add_action(TimerAction(period=3.0, actions=[web_pointcloud_bridge]))
    

    
    # Add all nodes to the launch description
    ld.add_action(
        TimerAction(period=5.0, actions=[tf_publisher]),  # 延迟启动，确保其他节点先启动
    )
    ld.add_action(
        xtd2_communication,
    )
    
    # 当wait_for_px4_odom成功完成后，启动ego_planner_launch
    # 在真实飞机环境下，同时启动super_lio
    if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
        # 模拟环境
        ego_planner_event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_px4_odom,
                on_exit=[
                    # super_lio_launch,
                    ego_planner_launch
                ]
            )
        )
    else:
        # 真实飞机环境
        on_exit_actions = [
            super_lio_launch,
            ego_planner_launch,
        ]
        if not build_map_mode:
            on_exit_actions.append(TimerAction(period=7.0, actions=[lidar_localization_launch]))
        ego_planner_event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_px4_odom,
                on_exit=on_exit_actions
            )
        )
    ld.add_action(ego_planner_event_handler)

    return ld
