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
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('/move_base_simple/goal', '/goal_pose'),  # RVIZ目标点话题
        ],
        parameters=[
            # 基本参数
            {'manager/drone_id': drone_id},
            
            # FSM 参数
            {'fsm/flight_type': 1},  # PRESET_TARGET模式
            {'fsm/thresh_replan_time': 0.5},  # 重规划时间阈值
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
            {'grid_map/map_size_x': 200.0},  # 地图X轴大小
            {'grid_map/map_size_y': 200.0},  # 地图Y轴大小
            {'grid_map/map_size_z': 40.0},  # 地图Z轴大小
            {'grid_map/local_update_range_x': 8.0},  # 局部更新范围X
            {'grid_map/local_update_range_y': 8.0},  # 局部更新范围Y
            {'grid_map/local_update_range_z': 4.0},  # 局部更新范围Z
            {'grid_map/obstacles_inflation': 0.2},  # 障碍物膨胀半径
            {'grid_map/local_map_margin': 5},  # 局部地图边界
            {'grid_map/ground_height': 0.0},  # 地面高度
            {'grid_map/p_occ': 0.2185},  # 占用概率
            {'grid_map/visualization_truncate_height': 10.0},  # 可视化截断高度
            {'grid_map/frame_id': [ namespace, '/StereoOV7251']},  # 地图坐标系
            
            # 规划器参数
            {'planning/max_vel': 2.0},  # 最大速度
            {'planning/max_acc': 6.0},  # 最大加速度
            {'planning/optimization_iters': 50},  # 优化迭代次数

            # manager参数 - 确保与planner_manager兼容
            {'manager/max_vel': 2.0},
            {'manager/max_acc': 6.0},
            {'manager/max_jerk': 10.0},
            {'manager/control_points_distance': 0.5},
            {'manager/planning_horizon': 7.5},
            
            # 点云处理参数
            {'grid_map/depth_filter_margin': 1},  # 深度滤波边界
            {'grid_map/skip_pixel': 2},  # 像素跳转
            {'grid_map/depth_scale': 1.0},  # 深度缩放
        ]
    )
    
    # 轨迹服务器节点
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        namespace=namespace,
        output='screen',
        remappings=[
            ('position_cmd', 'position_cmd'),
            ('traj_start_trigger', 'traj_start_trigger'),
            ('odom', ['/', namespace, '/odometry']),
        ]
    )
    
    # RVIZ启动（延迟启动，确保其他节点先启动）
    rviz_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', 
             PathJoinSubstitution([
                 FindPackageShare('ego_planner'),
                 'launch',
                 'default.rviz'
             ])],
        output='screen',
        name='rviz2'
    )
    
    # 创建启动描述
    ld = LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('namespace', default_value='x500_depth_0'),
        DeclareLaunchArgument('drone_id', default_value='0'),
        
        # 启动ego-planner节点
        ego_planner_node,
        
        # 延迟启动轨迹服务器
        TimerAction(period=2.0, actions=[traj_server_node]),
        
        # 延迟启动RVIZ
        # TimerAction(period=5.0, actions=[rviz_launch]),
    ])
    
    return ld