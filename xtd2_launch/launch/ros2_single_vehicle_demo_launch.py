import platform
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 检查主机名，设置默认namespace
hostname = platform.node()
if hostname == 'ywj-B250-D3A':
    default_namespace = '/x500_depth_0/'
    use_sim_time = True
    use_sim_time_str = 'true'
else:
    default_namespace = '/'
    use_sim_time = False
    use_sim_time_str = 'false'  


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', 
                    # default_value='aruco', 
                    # default_value='tugbot_warehouse',
                    default_value='ego',

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
        cmd=["MicroXRCEAgent udp4 -p 8888"],
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
                'model': 'gz_x500_depth',
                'id': '0',
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
            "--namespace", LaunchConfiguration('namespace'),
            "--debug", "true"
        ]
    )

    ####################
    # TF Publisher Node #
    ####################
    tf_publisher = ExecuteProcess(
        cmd=["ros2", "run", "xtd2_launch", "tf_publisher"],
        output='screen',
        name='tf_publisher',
        shell=False
    )

    #######################
    # EGO Planner Launch #
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

    # 根据主机名决定启动哪些组件
    # Add all nodes to the launch description
    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
        TimerAction(period=5.0, actions=[tf_publisher]),  # 延迟启动，确保其他节点先启动
        xtd2_communication,
        TimerAction(period=15.0, actions=[ego_planner_launch]),  # EGO Planner延迟启动
    ])
    
    
    # 当主机为ywj-B250-D3A时，启动整套px4模拟
    if hostname == 'ywj-B250-D3A':
        
        ld.add_action(world_launch)  # 启动Gazebo模拟环境
        ld.add_action(xrce_dds_process)  # 启动XRCE-DDS Agent
        ld.add_action(TimerAction(period=15.0, actions=[spawn]))  # 启动模型、PX4 SITL和ROS-Gazebo桥接
        ld.add_action(TimerAction(period=10.0, actions=[rviz_node]))  # 启动RViz可视化

    return ld