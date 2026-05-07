#!/usr/bin/env python3

import platform
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

# 检查主机名，设置默认namespace
hostname = platform.node()
if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
    default_namespace = '/x500_depth_0/'
    
else:
    default_namespace = '/'
    


def generate_launch_description():
    # 配置参数 - 针对Gazebo仿真环境
    namespace = LaunchConfiguration('namespace', default=default_namespace)
    drone_id = LaunchConfiguration('drone_id', default=0)

    map_size_x = LaunchConfiguration('map_size_x', default=250.0)
    map_size_y = LaunchConfiguration('map_size_y', default=250.0)
    map_size_z = LaunchConfiguration('map_size_z', default=20.0)

    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=0.5)
    
    # 根据主机名决定是否使用仿真时间
    if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
        default_use_sim_time = 'true'
        odom_world_topic = '/x500_depth_0/odometry'
        grid_map_cloud_topic = '/livox_down/lidar'  #'/x500_depth_0/StereoOV7251/pointcloud'
        grid_map_pose_topic = '/x500_depth_0/StereoOV7251/pose'
    else:
        default_use_sim_time = 'false'
        odom_world_topic = [namespace, TextSubstitution(text='lio/odom')]
        grid_map_cloud_topic = [namespace, TextSubstitution(text='lio/cloud_world')]
        grid_map_pose_topic = [namespace, TextSubstitution(text='mid360/pose')]
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=default_use_sim_time)
    
    # 路径点参数 - 根据Gazebo环境调整
    point_num = LaunchConfiguration('point_num', default=4)
    point0_x = LaunchConfiguration('point0_x', default=-1.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=2.0)
    point1_x = LaunchConfiguration('point1_x', default=-1.0)
    point1_y = LaunchConfiguration('point1_y', default=0.0)
    point1_z = LaunchConfiguration('point1_z', default=3.0)
    point2_x = LaunchConfiguration('point2_x', default=1.0)
    point2_y = LaunchConfiguration('point2_y', default=1.0)
    point2_z = LaunchConfiguration('point2_z', default=2.0)
    point3_x = LaunchConfiguration('point3_x', default=-1.0)
    point3_y = LaunchConfiguration('point3_y', default=1.0)
    point3_z = LaunchConfiguration('point3_z', default=2.0)
    point4_x = LaunchConfiguration('point4_x', default=0.0)
    point4_y = LaunchConfiguration('point4_y', default=0.0)
    point4_z = LaunchConfiguration('point4_z', default=2.0)

    # Ego Planner 节点配置 - 针对Gazebo环境
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        namespace=namespace,
        output='screen',
        prefix=['taskset -c 5,6'],   # 绑定 CPU 4
        # 重新映射话题以匹配Gazebo环境
        remappings=[
            ('odom_world', odom_world_topic),  # 使用Gazebo发布的里程计数据
            ('grid_map/cloud', grid_map_cloud_topic),  # 使用无人机的深度相机点云
            # ('grid_map/depth', [namespace, 'StereoOV7251/depth']),  # 使用无人机的深度相机深度图像
            ('grid_map/odom', odom_world_topic),  # 里程计数据用于地图构建
            ('grid_map/pose', grid_map_pose_topic),  # 位姿数据用于地图构建
            # ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate']),

            ('planning/bspline', [namespace, 'planning/bspline']),
            ('planning/data_display', [namespace, 'planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            ('/move_base_simple/goal', '/goal_pose_3d'),  # RVIZ目标点话题
            # ('goal_point', '/goal_pose_3d'),
        ],
        parameters=[
            # 基本参数
            {'use_sim_time': use_sim_time},
            
            # FSM 参数
            {'fsm/flight_type': 1},  # PRESET_TARGET模式2
            {'fsm/thresh_replan_time': 1.0},  # 重规划时间阈值  0.2
            {'fsm/thresh_no_replan_meter': 1.0},  # 重规划距离阈值 0.3
            {'fsm/planning_horizon': 7.5},  # 规划视野
            {'fsm/planning_horizen_time': 5.0},  # 规划时间视野  5.0
            {'fsm/emergency_time': 1.0},  # 紧急情况处理时间
            {'fsm/same_point_replan_time_threshold': 10.0},  # 相同目标点重规划时间阈值
            {'fsm/realworld_experiment': True},  # 仿真模式  False
            {'fsm/fail_safe': True},  # 启用安全保护
            {'fsm/frame_id': "world"},  # 目标坐标系frame_id

            # 路径点参数
            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},
            
            # 网格地图参数 - 针对Gazebo环境
            {'grid_map/resolution': 0.2},  # 地图分辨率   0.2
            {'grid_map/map_size_x': map_size_x},  # 地图X轴大小
            {'grid_map/map_size_y': map_size_y},  # 地图Y轴大小
            {'grid_map/map_size_z': map_size_z},  # 地图Z轴大小
            {'grid_map/local_update_range_x': 10.0},  # 局部更新范围X  10.0
            {'grid_map/local_update_range_y': 16.0},  # 局部更新范围Y  15.0
            {'grid_map/local_update_range_z': 8.0},  # 局部更新范围Z   8.0  
            {'grid_map/obstacles_inflation': 0.4},  # 障碍物膨胀半径 0.6
            {'grid_map/local_map_margin': 2},  # 局部地图边界
            {'grid_map/ground_height': -1.5},  # 地面高度

            # depth filter - 深度滤波器参数
            {'grid_map/use_depth_filter': False},  # 启用深度滤波
            {'grid_map/depth_filter_tolerance': 0.15},  # 深度滤波容差
            {'grid_map/depth_filter_maxdist': 20.0},  # 深度滤波最大距离
            {'grid_map/depth_filter_mindist': 1.0},  # 深度滤波最小距离
            # 点云处理参数
            {'grid_map/depth_filter_margin': 1},  # 深度滤波边界
            {'grid_map/skip_pixel': 10},  # 像素跳转
            {'grid_map/depth_scale': 1.0},  # 深度缩放

            # local fusion
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.2185},  # 占用概率
            {'grid_map/min_ray_length': 0.5},
            {'grid_map/max_ray_length': 30.0},

            {'grid_map/virtual_ceil_height': 30.0},  # 虚拟天花板高度
            {'grid_map/pose_type': 1},  # 位姿类型
            {'grid_map/show_occ_time': False},  # 显示占用时间
            {'grid_map/visualization_truncate_height': 9.0},  # 可视化截断高度
            {'grid_map/frame_id': 'world'},  # 地图坐标系
            
            {'grid_map/reset_buffer_enabled': True},  # 重置缓冲区是否启用
            {'grid_map/origin_safe_zone_enabled': True},  # 启用起始安全安全区域
            {'grid_map/origin_safe_zone_size_x': 2.0},
            {'grid_map/origin_safe_zone_size_y': 2.0},
            {'grid_map/origin_safe_zone_size_z': 2.0},
            
            # 规划器参数
            {'planning/max_vel': max_vel},  # 最大速度
            {'planning/max_acc': max_acc},  # 最大加速度
            {'planning/optimization_iters': 10},  # 优化迭代次数

            # manager参数 - 确保与planner_manager兼容
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 10.0},  # 最大加加速度
            {'manager/control_points_distance': 0.5},  # 控制点距离
            {'manager/feasibility_tolerance': 0.05},  # 可行性容差
            {'manager/planning_horizon': 7.5},  # 规划视野
            {'manager/use_distinctive_trajs': False},  # 使用不同的轨迹
            {'manager/drone_id': drone_id},  # 无人机ID


             # Trajectory optimization parameters - 轨迹优化参数
            {'optimization/lambda_smooth': 1.0},  # 平滑性权重
            {'optimization/lambda_collision': 8.0},  # 碰撞避免权重
            {'optimization/lambda_feasibility': 0.5},  # 可行性权重
            {'optimization/lambda_fitness': 1.0},  # 适应性权重
            {'optimization/dist0': 0.5},  # 初始距离
            {'optimization/swarm_clearance': 0.5},  # 群体间隙
            {'optimization/max_vel': max_vel},  # 优化最大速度
            {'optimization/max_acc': max_acc},  # 优化最大加速度
            
            # B-Spline parameters - B样条参数
            {'bspline/limit_vel': max_vel},  # B样条速度限制
            {'bspline/limit_acc': max_acc},  # B样条加速度限制
            {'bspline/limit_ratio': 1.1},  # B样条限制比例

            # Object prediction parameters - 目标预测参数
            {'prediction/obj_num': 1},  # 目标数量
            {'prediction/lambda': 1.0},  # 预测权重系数
            {'prediction/predict_rate': 1.0}  # 预测频率
        ]
    )
    
    # 轨迹服务器节点
    traj_server_node = Node(
        package='ego_planner',
        executable='xtd2_traj_server',
        name='xtd2_traj_server',
        namespace=namespace,
        output='screen',
        parameters=[
            # 基本参数
            {'use_sim_time': use_sim_time},
            {'traj_server/time_forward': 1.0},
            {'ros_ns': namespace},
            # Safe zone descent参数
            {'safe_zone_descent/enabled': True},
            {'safe_zone_descent/speed': 0.5},
            {'safe_zone_descent/zone_size_x': 2.0},
            {'safe_zone_descent/zone_size_y': 2.0},
            {'safe_zone_descent/zone_size_z': 2.0},
            {'safe_zone_descent/position_threshold': 0.05},
        ],
        remappings=[
            # ('position_cmd', 'position_cmd'),
            # ('traj_start_trigger', 'traj_start_trigger'),
            # ('odom', [namespace, 'odometry']),
            ('/xtdrone2/planning/cmd_pose_local_ned', ['/xtdrone2', namespace, 'cmd_pose_local_ned']),
            ('planning/bspline', [namespace, 'planning/bspline'])
        ]
    )

    interactive_marker_node = ExecuteProcess(
        cmd=["ros2", "run", "xtd2_launch", "goal_pose_marker"],
        output='screen',
        name='interactive_marker_node',
        shell=False
    )
    
    # RVIZ启动（延迟启动，确保其他节点先启动）
    rviz_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', 
             PathJoinSubstitution([
                 FindPackageShare('ego_planner'),
                 'launch',
                 'egoplanner.rviz'
             ])],
        output='screen',
        name='rviz2'
    )
    
    # 创建启动描述
    ld = LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('namespace', default_value=default_namespace),
        DeclareLaunchArgument('drone_id', default_value='0'),

        interactive_marker_node,

        # 启动ego-planner节点
        ego_planner_node,
        
        # 延迟启动轨迹服务器
        TimerAction(period=2.0, actions=[traj_server_node]),
        
        # 延迟启动RVIZ
        # TimerAction(period=5.0, actions=[rviz_launch]),
    ])
    
    return ld
