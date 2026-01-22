from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument('world_name', 
                    # default_value='aruco', 
                    default_value='tugbot_warehouse',
                    description='Name of the world to launch (without .sdf)')
    model_name_arg = DeclareLaunchArgument('model_name', default_value='gz_x500_depth', description='Name of the model to spawn')
    id_arg = DeclareLaunchArgument('id', default_value='0', description='ID of the model to spawn')
    name_space_arg = DeclareLaunchArgument('namespace', default_value='x500_depth_0', description='ROS namespace for the model')


    
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
                'namespace': 'x500_depth_0',
            }.items()
        )

    ####################
    # TF Publisher Node #
    ####################
    tf_publisher = ExecuteProcess(
        cmd=["python3", "/home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_launch/launch/tf_publisher.py"],
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
            'use_sim_time': 'true',
        }.items()
    )

    # Add all nodes to the launch description
    ld = LaunchDescription([
        world_name_arg,
        model_name_arg,
        id_arg,
        name_space_arg,
        world_launch,
        xrce_dds_process,
        TimerAction(period=10.0, actions=[spawn]),
        TimerAction(period=15.0, actions=[tf_publisher]),  # 延迟启动，确保其他节点先启动
        # TimerAction(period=20.0, actions=[ego_planner_launch])  # EGO Planner延迟启动，确保PX4和Gazebo完全就绪
    ])

    return ld