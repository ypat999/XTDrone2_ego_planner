#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 配置参数
    namespace = LaunchConfiguration('namespace', default='x500_depth_0')
    
    # Ego Planner 节点配置
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        namespace=namespace,
        output='screen',
        # 重新映射话题以匹配当前环境
        remappings=[
            ('odom_world', '/x500_depth_0/odometry'),  # 使用Gazebo发布的里程计数据
            ('grid_map/cloud', '/x500_depth_0/StereoOV7251/pointcloud'),  # 深度相机点云
            ('grid_map/odom', '/x500_depth_0/odometry'),  # 里程计数据用于地图构建
            ('planning/bspline', 'planning/bspline'),
            ('planning/data_display', 'planning/data_display'),
            ('/move_base_simple/goal', '/goal_pose'),  # RVIZ目标点话题
        ],
        parameters=[
            # FSM 参数
            {'fsm/flight_type': 1},  # MANUAL_TARGET模式，通过RVIZ设置目标（1=MANUAL_TARGET, 2=PRESET_TARGET）
            {'fsm/thresh_replan_time': 0.5},  # 重规划时间阈值
            {'fsm/thresh_no_replan_meter': 0.3},  # 重规划距离阈值
            {'fsm/planning_horizon': 10.0},  # 规划视野
            {'fsm/planning_horizen_time': 3.0},  # 规划时间视野
            {'fsm/emergency_time': 1.0},  # 紧急情况处理时间
            {'fsm/realworld_experiment': False},  # 仿真模式
            {'fsm/fail_safe': True},  # 启用安全保护
            {'fsm/waypoint_num': 0},  # 禁用预设路径点
            
            # 网格地图参数
            {'grid_map/resolution': 0.1},  # 地图分辨率
            {'grid_map/map_size_x': 50.0},  # 地图X轴大小
            {'grid_map/map_size_y': 50.0},  # 地图Y轴大小
            {'grid_map/map_size_z': 20.0},  # 地图Z轴大小
            {'grid_map/local_update_range_x': 8.0},  # 局部更新范围X
            {'grid_map/local_update_range_y': 8.0},  # 局部更新范围Y
            {'grid_map/local_update_range_z': 4.0},  # 局部更新范围Z
            {'grid_map/obstacles_inflation': 0.2},  # 障碍物膨胀半径
            {'grid_map/local_map_margin': 5},  # 局部地图边界
            {'grid_map/ground_height': 0.0},  # 地面高度
            
            # 规划器参数
            {'planning/max_vel': 2.0},  # 最大速度
            {'planning/max_acc': 3.0},  # 最大加速度
            {'planning/optimization_iters': 50},  # 优化迭代次数
            
            # 点云处理参数
            {'grid_map/depth_filter_margin': 1},  # 深度滤波边界
            {'grid_map/skip_pixel': 2},  # 像素跳转
            {'grid_map/depth_scale': 1.0},  # 深度缩放
        ]
    )
    
    # 轨迹服务器节点（用于可视化轨迹）
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        namespace=namespace,
        output='screen',
        remappings=[
            ('position_cmd', 'position_cmd'),
            ('traj_start_trigger', 'traj_start_trigger'),
            ('odom', '/x500_depth_0/odometry'),
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
        
        # 启动ego-planner节点
        ego_planner_node,
        
        # 延迟启动轨迹服务器
        TimerAction(period=2.0, actions=[traj_server_node]),

        TimerAction(period=2.0, actions=[voxel_grid_downsample_node]),
        
        # 延迟启动RVIZ
        # TimerAction(period=5.0, actions=[rviz_launch]),
    ])
    
    return ld