#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 配置参数 - 针对Gazebo仿真环境
    namespace = LaunchConfiguration('namespace', default='x500_depth_0')
    drone_id = LaunchConfiguration('drone_id', default=0)

    map_size_x = LaunchConfiguration('map_size_x', default=200.0)
    map_size_y = LaunchConfiguration('map_size_y', default=200.0)
    map_size_z = LaunchConfiguration('map_size_z', default=40.0)

    max_vel = LaunchConfiguration('max_vel', default=3.0)
    max_acc = LaunchConfiguration('max_acc', default=0.5)
    
    # 启用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
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
        # 重新映射话题以匹配Gazebo环境
        remappings=[
            ('odom_world', ['/', namespace, '/odometry']),  # 使用Gazebo发布的里程计数据
            ('grid_map/cloud', ['/', namespace, '/StereoOV7251/pointcloud']),  # 使用无人机的深度相机点云
            # ('grid_map/depth', ['/', namespace, '/StereoOV7251/depth']),  # 使用无人机的深度相机深度图像
            ('grid_map/odom', ['/', namespace, '/odometry']),  # 里程计数据用于地图构建
            ('grid_map/pose', ['/', namespace, '/StereoOV7251/pose']),  # 位姿数据用于地图构建
            # ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate']),

            ('planning/bspline', ['/', namespace, '/planning/bspline']),
            ('planning/data_display', ['/', namespace, '/planning/data_display']),
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
            {'fsm/thresh_replan_time': 0.2},  # 重规划时间阈值
            {'fsm/thresh_no_replan_meter': 0.3},  # 重规划距离阈值
            {'fsm/planning_horizon': 7.5},  # 规划视野
            {'fsm/planning_horizen_time': 3.0},  # 规划时间视野
            {'fsm/emergency_time': 1.0},  # 紧急情况处理时间
            {'fsm/realworld_experiment': False},  # 仿真模式
            {'fsm/fail_safe': True},  # 启用安全保护

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
            {'grid_map/resolution': 0.2},  # 地图分辨率
            {'grid_map/map_size_x': map_size_x},  # 地图X轴大小
            {'grid_map/map_size_y': map_size_y},  # 地图Y轴大小
            {'grid_map/map_size_z': map_size_z},  # 地图Z轴大小
            {'grid_map/local_update_range_x': 10.0},  # 局部更新范围X
            {'grid_map/local_update_range_y': 15.0},  # 局部更新范围Y
            {'grid_map/local_update_range_z': 5.0},  # 局部更新范围Z
            {'grid_map/obstacles_inflation': 0.6},  # 障碍物膨胀半径
            {'grid_map/local_map_margin': 5},  # 局部地图边界
            {'grid_map/ground_height': -0.01},  # 地面高度

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
            {'grid_map/max_ray_length': 14.5},

            {'grid_map/virtual_ceil_height': 40.0},  # 虚拟天花板高度
            {'grid_map/pose_type': 1},  # 位姿类型
            {'grid_map/show_occ_time': False},  # 显示占用时间
            {'grid_map/visualization_truncate_height': 40.0},  # 可视化截断高度
            {'grid_map/frame_id': 'world'},  # 地图坐标系

            
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
            {'manager/use_distinctive_trajs': True},  # 使用不同的轨迹
            {'manager/drone_id': drone_id},  # 无人机ID


             # Trajectory optimization parameters - 轨迹优化参数
            {'optimization/lambda_smooth': 1.0},  # 平滑性权重
            {'optimization/lambda_collision': 2.0},  # 碰撞避免权重
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
            {'prediction/obj_num': 10},  # 目标数量
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
        ],
        remappings=[
            # ('position_cmd', 'position_cmd'),
            # ('traj_start_trigger', 'traj_start_trigger'),
            # ('odom', ['/', namespace, '/odometry']),
            ('/xtdrone2/planning/cmd_pose_local_ned', '/xtdrone2/x500_depth_0/cmd_pose_local_ned'),
            ('planning/bspline', '/x500_depth_0/planning/bspline')
        ]
    )

    interactive_marker_node = ExecuteProcess(
        cmd=["python3", "/home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_launch/launch/goal_pose_interactive_marker.py"],
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
        DeclareLaunchArgument('namespace', default_value='x500_depth_0'),
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