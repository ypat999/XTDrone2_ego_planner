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
            # build_map_mode 时同时输出 SC-PGO 兼容数据(lio.sc_pgo.enable)
            'sc_pgo': 'true' if build_map_mode else 'false',

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

    #######################
    # Obstacle Distance Publisher (mid360 -> PX4 Collision Prevention)
    #######################
    # tf 外参模式默认关闭; real 分支开启后从 /tf 自动获取, 忽略 quat/trans
    cp_tf_parent = ''
    cp_tf_child = ''
    cp_use_world = False
    if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
        # 仿真: x500_depth 前向 mid360 (/livox/lidar), 相对机体前倾 30°(RPY=0,30,0), 平移(0.1,0,0.3)
        # 雷达系->机体系FRD 变换四元数由 R=Rx(180)*Ry(30) 计算得到
        cp_cloud_topic = '/livox/lidar'
        cp_lidar_quat = [0.0, 0.9659, 0.0, 0.2588]   # w,x,y,z
        cp_lidar_trans = [0.1, 0.0, -0.3]
    else:
        # 真实: /livox/lidar 是 livox CustomMsg(驱动 xfer_format=1), 本节点无法直接订阅
        # 改用 Super-LIO 输出的 lio/cloud_world (PointCloud2, world 系, 10Hz, 已去畸变)
        # world 模式: 高度带以 world z(相对飞机)衡量, 不随飞机俯仰倾斜;
        #   飞机位姿用 Super-LIO 动态 TF world->base_footprint(与 world 平行, 只有 yaw)
        cp_cloud_topic = 'lio/cloud_world'
        cp_use_world = True
        cp_lidar_quat = [1.0, 0.0, 0.0, 0.0]         # world 模式忽略
        cp_lidar_trans = [0.0, 0.0, 0.0]             # world 模式忽略
        # world 模式无需静态外参(位姿直接来自 world->base_footprint TF), 仅保留占位
        cp_tf_parent = 'base_footprint'
        cp_tf_child = ''

    obstacle_distance_publisher = Node(
        package='xtd2_communication',
        executable='obstacle_distance_publisher',
        name='obstacle_distance_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cloud_topic': cp_cloud_topic,
            'namespace': LaunchConfiguration('namespace'),
            'quat_wxyz': cp_lidar_quat,
            'trans_xyz': cp_lidar_trans,
            'tf_parent_frame': cp_tf_parent,
            'tf_child_frame': cp_tf_child,
            'use_world_cloud': cp_use_world,
            'use_sim_time': use_sim_time,
        }],
        prefix=['taskset -c 0,1,2,3'],   # 绑定 CPU
    )

    ##############################
    # PX4 CP 参数设置 (仅仿真)
    # 通过 px4-param 客户端写入运行中的 PX4 SITL 实例
    # CP_GO_NO_DATA=1: 无避障数据时也允许飞行(移动到未知空间)
    # CP_GUIDE_ANG=0: 避障时禁用偏航修正, 只做平移避让(避免机头乱转)
    # COM_OBS_AVOID=0: 我们走机载原生 CP(obstacle_distance->collision_prevention),
    #                   不依赖外部避障栈心跳; 若为1会报 "Avoidance system not ready" 无法解锁
    # MPC_POS_MODE=3: 关键! 默认4=加速控制(FlightTaskManualAcceleration, 无CP),
    #                 Position 模式只有位置控制类任务(0=ManualPosition / 3=SmoothVel)才运行
    #                 Collision Prevention; 用3保留平滑手感
    # 注意: 真实飞行时这些参数需在 QGC 中设置, 会持久化到飞控
    ##############################
    px4_cp_param_set = ExecuteProcess(
        cmd=[
            'bash', '-c',
            "BIN=$HOME/git/PX4-Autopilot/build/px4_sitl_default/bin/px4-param; "
            "echo '设置 PX4 CP 参数 (CP_GO_NO_DATA=1: 无数据时允许飞行)...'; "
            "$BIN --instance ${PX4_ID} set COM_OBS_AVOID 0; "
            "$BIN --instance ${PX4_ID} set CP_GO_NO_DATA 1; "
            "echo 'CP_GO_NO_DATA =' $($BIN --instance ${PX4_ID} show CP_GO_NO_DATA); "
            "$BIN --instance ${PX4_ID} set CP_GUIDE_ANG 0; "
            "echo 'CP_GUIDE_ANG =' $($BIN --instance ${PX4_ID} show CP_GUIDE_ANG); "
            "$BIN --instance ${PX4_ID} set CP_DIST 1.0; "
            "echo 'CP_DIST =' $($BIN --instance ${PX4_ID} show CP_DIST); "
            "$BIN --instance ${PX4_ID} set MPC_POS_MODE 3; "
            "echo 'MPC_POS_MODE =' $($BIN --instance ${PX4_ID} show MPC_POS_MODE)"
        ],
        env={'PX4_ID': LaunchConfiguration('id')},
        output='screen',
        name='px4_cp_param_set',
        shell=False,
    )

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
    ld.add_action(obstacle_distance_publisher)  # mid360 点云 -> PX4 Collision Prevention
    
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
                    px4_cp_param_set,   # 设置 PX4 CP 参数(无数据时允许飞行)
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
