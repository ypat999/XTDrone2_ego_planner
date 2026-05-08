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
            # FSM (Finite State Machine) 参数 - 有限状态机控制参数
            {'fsm/flight_type': 1},  # 飞行类型: 1=PRESET_TARGET(预设目标点模式), 2=MANUAL_TARGET(手动目标点模式)
                                       # PRESET_TARGET模式会使用预设的航点序列，MANUAL_TARGET模式通过RViz交互选择目标点
            
            {'fsm/thresh_replan_time': 1.0},  # 重规划时间阈值(秒): 当距离上次规划超过此时间时触发重规划
                                               # 值越小重规划越频繁，路径更优但计算开销更大；值越大计算开销小但路径可能不够优化
                                               # 推荐范围: 0.5-2.0秒，实时性要求高时设小值
            
            {'fsm/thresh_no_replan_meter': 1.0},  # 重规划距离阈值(米): 当无人机偏离当前轨迹超过此距离时触发重规划
                                                   # 值越小对轨迹跟踪精度要求越高，重规划更频繁；值越大容忍度越高
                                                   # 推荐范围: 0.5-2.0米，复杂环境建议设小值
            
            {'fsm/planning_horizon': 7.5},  # 规划视野半径(米): 规划器考虑的局部地图范围
                                             # 值越大规划越长远但计算开销越大；值越小规划越局部但响应更快
                                             # 推荐范围: 5.0-15.0米，大场景建议增大
            
            {'fsm/planning_horizen_time': 5.0},  # 规划时间视野(秒): 规划器考虑的未来时间范围
                                                  # 影响轨迹预测长度，值越大轨迹越长但不确定性增加
                                                  # 推荐范围: 3.0-8.0秒，高速飞行建议增大
            
            {'fsm/emergency_time': 1.0},  # 紧急情况处理时间(秒): 检测到碰撞风险时的紧急避障响应时间
                                           # 值越小反应越快但可能过于激进；值越大反应平缓但可能不够及时
                                           # 推荐范围: 0.5-2.0秒
            
            {'fsm/same_point_replan_time_threshold': 10.0},  # 相同目标点重规划时间阈值(秒): 对同一目标点多久后允许重新规划
                                                              # 防止对静态目标点频繁重规划，节省计算资源
                                                              # 推荐范围: 5.0-20.0秒
            
            {'fsm/realworld_experiment': True},  # 实验模式: True=真实世界实验, False=仿真实验
                                                  # 真实世界模式会启用更多安全检查和传感器融合
                                                  # 仿真模式可以简化某些处理流程
            
            {'fsm/fail_safe': True},  # 安全保护开关: True=启用故障安全保护, False=禁用
                                       # 启用后会在检测到异常时自动悬停或返航，提高安全性
                                       # 建议真实实验时始终启用
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

            # Grid Map 深度滤波参数 - 点云预处理和滤波设置
            {'grid_map/use_depth_filter': False},  # 深度滤波开关: True=启用深度滤波, False=禁用
                                                     # 启用后会过滤掉不可靠的深度测量，提高地图质量但增加计算开销
                                                     # 推荐在深度相机噪声较大时启用
            
            {'grid_map/depth_filter_tolerance': 0.15},  # 深度滤波容差(米): 深度测量的容许误差范围
                                                         # 值越小滤波越严格，保留的点越少但质量更高；值越大保留更多点但可能包含噪声
                                                         # 推荐范围: 0.1-0.3米，根据深度相机精度调整
            
            {'grid_map/depth_filter_maxdist': 20.0},  # 深度滤波最大距离(米): 过滤掉超过此距离的深度测量
                                                       # 超出此距离的测量通常不可靠，建议根据传感器有效范围设置
                                                       # 推荐范围: 10.0-30.0米
            
            {'grid_map/depth_filter_mindist': 1.0},  # 深度滤波最小距离(米): 过滤掉小于此距离的深度测量
                                                      # 近距离测量可能受传感器盲区影响，建议保留安全距离
                                                      # 推荐范围: 0.5-2.0米
            
            # 点云处理参数
            {'grid_map/depth_filter_margin': 1},  # 深度滤波边界(像素): 图像边缘裁剪宽度
                                                   # 去除图像边缘的不稳定测量，减少边界噪声
                                                   # 推荐范围: 1-5像素
            
            {'grid_map/skip_pixel': 10},  # 像素跳转步长: 每隔多少像素采样一次
                                           # 值越大点云越稀疏但处理速度越快；值越小点云越密集但计算开销越大
                                           # 推荐范围: 5-20像素，实时性要求高时增大
            
            {'grid_map/depth_scale': 1.0},  # 深度缩放因子: 对深度值进行缩放
                                             # 用于调整深度单位，通常保持1.0即可
                                             # 特殊传感器可能需要调整

            # Local Fusion 局部融合参数 - 贝叶斯占用网格更新参数
            {'grid_map/p_hit': 0.65},  # 占用概率增量: 当射线击中障碍物时，该栅格的占用概率增加量
                                        # 值越大对障碍物越敏感，地图更新越快；值越小更新越保守
                                        # 推荐范围: 0.55-0.75，环境变化快时增大
            
            {'grid_map/p_miss': 0.35},  # 空闲概率增量: 当射线穿过栅格时，该栅格的占用概率减少量
                                          # 值越大对空闲空间越敏感；值越小更新越保守
                                          # 推荐范围: 0.25-0.45，应小于p_hit
            
            {'grid_map/p_min': 0.12},  # 占用概率最小值: 栅格占用概率的下限
                                        # 防止概率过低导致无法恢复，保持一定的环境记忆
                                        # 推荐范围: 0.05-0.20
            
            {'grid_map/p_max': 0.90},  # 占用概率最大值: 栅格占用概率的上限
                                        # 防止概率过高导致无法更新，保持一定的灵活性
                                        # 推荐范围: 0.85-0.95
            
            {'grid_map/p_occ': 0.2185},  # 占用判定阈值: 栅格被认为占用时的概率阈值
                                           # 值越小越容易判定为障碍物，更安全但可能误判；值越大越保守
                                           # 推荐范围: 0.15-0.30，安全要求高时减小
            
            {'grid_map/min_ray_length': 0.5},  # 最小射线长度(米): 射线追踪的最小距离
                                                 # 过滤掉过近的测量，避免传感器噪声
                                                 # 推荐范围: 0.3-1.0米
            
            {'grid_map/max_ray_length': 30.0},  # 最大射线长度(米): 射线追踪的最大距离
                                                  # 限制计算范围，超出此距离的测量不参与更新
                                                  # 推荐范围: 20.0-50.0米，根据传感器范围调整

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
            # Manager 规划管理器参数 - 轨迹生成和管理设置
            {'manager/control_points_distance': 0.5},  # B样条控制点距离(米): 相邻控制点之间的最小距离
                                                         # 值越小轨迹越精细但控制点越多，计算开销越大；值越大轨迹越粗糙但计算更快
                                                         # 推荐范围: 0.3-1.0米，复杂环境建议减小
            
            {'manager/feasibility_tolerance': 0.05},  # 可行性容差: 轨迹可行性检查的容许误差
                                                       # 值越小对动力学约束要求越严格；值越大允许的偏差越大
                                                       # 推荐范围: 0.01-0.10，高精度控制时减小
            
            {'manager/planning_horizon': 7.5},  # 规划视野半径(米): 规划器考虑的局部空间范围
                                                 # 值越大规划越长远但计算开销越大；值越小规划越局部
                                                 # 推荐范围: 5.0-15.0米，应与fsm/planning_horizon一致
            
            {'manager/use_distinctive_trajs': False},  # 使用不同轨迹: True=生成多条候选轨迹, False=单轨迹优化
                                                        # 启用后会增加轨迹多样性但计算开销显著增加
                                                        # 推荐在复杂环境或需要避障时启用
            {'manager/drone_id': drone_id},  # 无人机ID


             # Trajectory Optimization 轨迹优化参数 - 代价函数权重设置
            {'optimization/lambda_smooth': 1.0},  # 平滑性权重: 控制轨迹的平滑程度
                                                    # 值越大轨迹越平滑但可能偏离最短路径；值越小轨迹更直接但可能不够平滑
                                                    # 推荐范围: 0.5-2.0，舒适度要求高时增大
            
            {'optimization/lambda_collision': 8.0},  # 碰撞避免权重: 控制轨迹与障碍物的安全距离
                                                      # 值越大离障碍物越远越安全但路径可能绕远；值越小路径更短但风险增加
                                                      # 推荐范围: 5.0-15.0，密集障碍环境建议增大
            
            {'optimization/lambda_feasibility': 0.5},  # 可行性权重: 控制轨迹满足动力学约束的程度
                                                        # 值越大对速度/加速度限制越严格；值越小允许更激进的机动
                                                        # 推荐范围: 0.3-1.0，高速飞行时增大
            
            {'optimization/lambda_fitness': 1.0},  # 适应性权重: 控制轨迹对目标点的追踪精度
                                                    # 值越大越趋向目标点；值越小允许更多偏离以优化其他目标
                                                    # 推荐范围: 0.5-2.0，精确到达要求高时增大
            
            {'optimization/dist0': 0.5},  # 初始安全距离(米): 优化开始时的障碍物安全距离阈值
                                           # 值越大初始轨迹离障碍物越远；值越小初始轨迹更接近障碍物
                                           # 推荐范围: 0.3-1.0米，安全裕度要求高时增大
            
            {'optimization/swarm_clearance': 0.5},  # 群体间隙(米): 多机编队时无人机之间的安全距离
                                                     # 值越大机间距离越远越安全但编队越松散；值越小编队越紧凑
                                                     # 推荐范围: 0.3-1.0米，单机运行时此参数不生效
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
            {'safe_zone_descent/speed': 0.2},
            {'safe_zone_descent/zone_size_x': 1.0},
            {'safe_zone_descent/zone_size_y': 1.0},
            {'safe_zone_descent/zone_size_z': 1.0},
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
