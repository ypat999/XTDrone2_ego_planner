"""
Multirotor Communication Module

This module provides communication functionalities for multirotor.

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

All rights reserved. This code is licensed under the MIT License.
"""

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleAttitudeSetpoint, VehicleStatus, VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from xtd2_msgs.srv import XTD2Cmd
from xtd2_msgs.msg import XTD2VehicleState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_geometry_msgs

import sys
import math
import numpy as np
from transforms3d.euler import quat2euler
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import argparse
import platform

from .coordinate_transform import CoordinateTransform

class MultirotorCommunication(Node):
    def __init__(self, model, id, namespace="", debug=False, allowarm=False):
        
        if model.startswith("gz_"):  # 删除gz_前缀
            model = model[3:]
        self.model = model

        self.id = int(id)
        self.debug = debug
        self.allowarm = allowarm

        self.namespace = namespace if namespace else f'{model}_{id}'

        # 生成有效的节点名称（只包含字母数字和下划线）
        node_name = self.namespace.strip('/') + '_communication'


        self.callback_stats = {
            'timer_callback': {'count': 0, 'total_time': 0.0},
            'vehicle_local_position_callback': {'count': 0, 'total_time': 0.0},
            'vehicle_global_position_callback': {'count': 0, 'total_time': 0.0},
            'vehicle_status_callback': {'count': 0, 'total_time': 0.0},
            'px4_odom_callback': {'count': 0, 'total_time': 0.0},
            'ros2_odom_callback': {'count': 0, 'total_time': 0.0},
            'cmd_pose_local_ned_callback': {'count': 0, 'total_time': 0.0},
            'cmd_pose_local_flu_callback': {'count': 0, 'total_time': 0.0},
            'cmd_trajectory_flu_callback': {'count': 0, 'total_time': 0.0},
            'cmd_vel_ned_callback': {'count': 0, 'total_time': 0.0},
            'cmd_vel_flu_callback': {'count': 0, 'total_time': 0.0},
            'cmd_accel_ned_callback': {'count': 0, 'total_time': 0.0},
            'cmd_accel_flu_callback': {'count': 0, 'total_time': 0.0},
            'cmd_attitude_flu_callback': {'count': 0, 'total_time': 0.0},
            'goal_marker_callback': {'count': 0, 'total_time': 0.0},
        }
        
        # 检查主机名，设置 use_sim_time（与 tf_publisher 保持一致）

        self.hostname = platform.node()
        if self.hostname == 'ywj-B250-D3A' or self.hostname == 'DESKTOP-ypat':
            use_sim_time = True
            odom_topic = self.namespace + 'odometry'
        else:
            use_sim_time = False
            odom_topic = "/lio/odom"
            
        super().__init__(node_name, parameter_overrides=[
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        self.OFFBOARD_STATE = "DISABLED"
        self.cmd = None
        self.cur_vehicle_local_position = None
        self.cur_vehicle_odometry = None
        self.cur_lio_vehicle_odometry = None
        self.cur_vehicle_global_position = None
        self.init_vehicle_local_position = None
        self.init_vehicle_global_position = None
        self.vehicle_status = None
        self.auto_switch_enabled = False
        self.last_goal_marker_time = None
        self.was_flying = False  # 记录之前是否在飞行状态
        self.landed_time = None  # 记录落地时间
        self.auto_switch_completed = False  # 记录自动切换是否已完成
        self.last_px4_odom_time = 0.0  # 上次 px4_odom_callback 调用时间
        self.last_ros2_odom_time = 0.0  # 上次 ros2_odom_callback 调用时间
        self.odom_callback_min_interval = 0.1  # 最小调用间隔 (秒)
        self.last_valid_enu_position = None  # 记录上一次有效的ENU位置

        # XTDrone2 Interface
        # 移除namespace前后的斜杠，避免重复
        clean_namespace = self.namespace.strip('/')
        if clean_namespace:
            xtdrone2_topic_prefix = f'/xtdrone2/{clean_namespace}/'
            dds_topic_prefix = f'/{clean_namespace}/'
        else:
            xtdrone2_topic_prefix = '/xtdrone2/'
            dds_topic_prefix = '/'

        print(f"XTDrone2 Topic Prefix: {xtdrone2_topic_prefix}")
        print(f"DDS Topic Prefix: {dds_topic_prefix}")
        
        self.create_subscription(Pose, xtdrone2_topic_prefix + 'cmd_pose_local_ned', self.cmd_pose_local_ned_callback, 10)  # geometry_msgs/Pose
        self.create_subscription(PoseStamped, xtdrone2_topic_prefix + 'cmd_pose_local_flu', self.cmd_pose_local_flu_callback, 10)  # geometry_msgs/PoseStamped
        self.create_subscription(PositionCommand, xtdrone2_topic_prefix + 'cmd_trajectory_flu', self.cmd_trajectory_flu_callback, 10)  # quadrotor_msgs/PositionCommand
        self.create_subscription(PoseStamped, xtdrone2_topic_prefix + 'cmd_vel_ned', self.cmd_vel_ned_callback, 10)  # geometry_msgs/PoseStamped
        self.create_subscription(Twist, xtdrone2_topic_prefix + 'cmd_vel_flu', self.cmd_vel_flu_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, xtdrone2_topic_prefix + 'cmd_accel_ned', self.cmd_accel_ned_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, xtdrone2_topic_prefix + 'cmd_accel_flu', self.cmd_accel_flu_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, xtdrone2_topic_prefix + 'cmd_attitude_flu', self.cmd_attitude_flu_callback, 10)  # geometry_msgs/Pose
        self.cmd_server = self.create_service(XTD2Cmd, xtdrone2_topic_prefix + 'cmd', self.cmd_callback)

        # DDS Interface
        self.create_subscription(VehicleLocalPosition, dds_topic_prefix + 'fmu/out/vehicle_local_position', self.vehicle_local_position_callback, QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
        self.create_subscription(VehicleGlobalPosition, dds_topic_prefix + 'fmu/out/vehicle_global_position', self.vehicle_global_position_callback, QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
        self.create_subscription(VehicleStatus, dds_topic_prefix + 'fmu/out/vehicle_status', self.vehicle_status_callback, QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
        self.create_subscription(VehicleOdometry, dds_topic_prefix + 'fmu/out/vehicle_odometry', self.px4_odom_callback, QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, dds_topic_prefix + 'fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, dds_topic_prefix + 'fmu/in/offboard_control_mode', 10)
        self.dds_trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, dds_topic_prefix + 'fmu/in/trajectory_setpoint', 10)
        self.dds_vehicle_attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint, dds_topic_prefix + 'fmu/in/vehicle_attitude_setpoint', 10)
        
        # PX4 visual odometry publisher
        self.px4_visual_pub = self.create_publisher(
            VehicleOdometry,
            dds_topic_prefix + 'fmu/in/vehicle_visual_odometry',
            QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        )
        
        # TF broadcaster for px4_odom
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Static TF broadcaster for world->px4_odom
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # TF buffer and listener for coordinate transformations
        # 优化 TF 缓冲区配置，提高实时性
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))  # 减少缓存时间到2秒
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)  # 启用独立线程
        
        # Goal marker subscription for automatic switching
        print(f"Allow Arm: {self.allowarm}")
        if self.allowarm:
            self.create_subscription(PoseStamped, '/goal_pose_3d', self.goal_marker_callback, 10)
        
        # Gazebo odometry subscription for PX4 visual odometry
        self.create_subscription(
            Odometry,
            odom_topic,
            self.ros2_odom_callback,
            10
        )

        self.timer_ = self.create_timer(0.05, self.timer_callback)

        # Debug publisher for vehicle state
        if self.debug:
            self.vehicle_state_publisher = self.create_publisher(XTD2VehicleState, xtdrone2_topic_prefix + 'debug/vehicle_state', 10)
            self.debug_timer = self.create_timer(0.1, self.publish_vehicle_state)  # 10Hz

        
        self.stats_start_time = time.time()
        self.stats_timer = self.create_timer(60.0, self.print_callback_stats)

        self.get_logger().info(f'{self.namespace} communication node started')
    
    def print_callback_stats(self):
        """每分钟输出回调函数统计信息"""
        elapsed_time = time.time() - self.stats_start_time
        self.get_logger().info(f'=== Callback Statistics (last {elapsed_time:.1f}s) ===')
        for name, stats in self.callback_stats.items():
            if stats['count'] > 0:
                avg_time = stats['total_time'] / stats['count'] * 1000
                freq = stats['count'] / elapsed_time
                self.get_logger().info(
                    f'  {name}: count={stats["count"]}, freq={freq:.1f}Hz, '
                    f'avg_time={avg_time:.3f}ms, total_time={stats["total_time"]*1000:.1f}ms'
                )
        self.callback_stats = {k: {'count': 0, 'total_time': 0.0} for k in self.callback_stats}
        self.stats_start_time = time.time()
    
    def timer_callback(self):
        start_time = time.time()
        # 检查自动切换状态
        if self.auto_switch_enabled:
            self.check_auto_switch_progress()
        
        # 检查无人机落地状态并自动解除arm
        self.check_landing_and_disarm()
        
        # 当OFFBOARD_STATE为ENABLED时，持续发送offboard心跳信号
        if self.OFFBOARD_STATE == "ENABLED":
            # 发送基础offboard控制模式（心跳信号）
            msg = OffboardControlMode()
            msg.timestamp = self.get_clock_microseconds()
            # 设置至少一个控制模式为True，否则PX4会拒绝offboard模式
            msg.position = True  # 设置为位置控制模式
            self.offboard_control_mode_pub.publish(msg)
            if self.cmd:
                self.cmd.timestamp = self.get_clock_microseconds()
                self.dds_trajectory_setpoint_pub.publish(self.cmd)
            self.callback_stats['timer_callback']['count'] += 1
            self.callback_stats['timer_callback']['total_time'] += time.time() - start_time
            return
        
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['timer_callback']['count'] += 1
            self.callback_stats['timer_callback']['total_time'] += time.time() - start_time
            return
        
        # Publish Offboard Control Mode (heartbeat)
        msg = OffboardControlMode()
        
        # Set control mode flags based on current OFFBOARD_STATE
        if self.OFFBOARD_STATE in ["POSE_LOCAL_NED", "POSE_LOCAL_FLU"]:
            msg.position = True
        elif self.OFFBOARD_STATE in ["VEL_NED", "VEL_FLU"]:
            msg.velocity = True
        elif self.OFFBOARD_STATE in ["ACCEL_NED", "ACCEL_FLU"]:
            msg.acceleration = True
        elif self.OFFBOARD_STATE == "ATTITUDE_FLU":
            msg.attitude = True
        
        # Publish control command if available
        if self.cmd:
            self.cmd.timestamp = self.get_clock_microseconds()
            if self.OFFBOARD_STATE in ["POSE_LOCAL_NED", "POSE_LOCAL_FLU", "VEL_NED", "VEL_FLU", "ACCEL_NED", "ACCEL_FLU"]:
                self.dds_trajectory_setpoint_pub.publish(self.cmd)
            elif self.OFFBOARD_STATE == "ATTITUDE_FLU":
                self.dds_vehicle_attitude_setpoint_pub.publish(self.cmd)
        
        # Always publish the offboard control mode (heartbeat)
        msg.timestamp = self.get_clock_microseconds()  
        self.offboard_control_mode_pub.publish(msg)
        self.callback_stats['timer_callback']['count'] += 1
        self.callback_stats['timer_callback']['total_time'] += time.time() - start_time
    
    def vehicle_local_position_callback(self, msg):
        start_time = time.time()
        if self.init_vehicle_local_position is None:
            self.init_vehicle_local_position = msg
        self.cur_vehicle_local_position = msg
        self.callback_stats['vehicle_local_position_callback']['count'] += 1
        self.callback_stats['vehicle_local_position_callback']['total_time'] += time.time() - start_time

    def vehicle_global_position_callback(self, msg):
        start_time = time.time()
        if self.init_vehicle_global_position is None:
            self.init_vehicle_global_position = msg
        self.cur_vehicle_global_position = msg
        self.callback_stats['vehicle_global_position_callback']['count'] += 1
        self.callback_stats['vehicle_global_position_callback']['total_time'] += time.time() - start_time

    def vehicle_status_callback(self, msg):
        start_time = time.time()
        self.vehicle_status = msg
        self.callback_stats['vehicle_status_callback']['count'] += 1
        self.callback_stats['vehicle_status_callback']['total_time'] += time.time() - start_time

    def px4_odom_callback(self, msg):
        """PX4 odometry callback - 发布 base_link -> px4_odom tf (相对变换)"""
        start_time = time.time()
        
        # 间隔限制检查
        current_time = time.time()
        if current_time - self.last_px4_odom_time < self.odom_callback_min_interval:
            # self.callback_stats['px4_odom_callback']['count'] += 1
            self.callback_stats['px4_odom_callback']['total_time'] += time.time() - start_time
            return
        self.last_px4_odom_time = current_time
        
        # PX4 odometry 表示无人机在 NED 坐标系中的位置
        # 我们需要发布 base_link -> px4_odom 的相对变换
        # 这个变换表示 px4_odom 在 base_link 坐标系中的位置
        self.cur_vehicle_odometry = msg

        # 读取 world->base_link 的 tf
        base_link_frame = self.namespace.lstrip('/') + 'base_link'
        px4_odom_frame = self.namespace.lstrip('/') + 'px4_odom'

        # 转换坐标系：NED -> ENU (位置)
        enu_position = CoordinateTransform.ned_to_enu_position(msg.position[0], msg.position[1], msg.position[2])
        # 转换姿态：FRD/NED -> FLU/ENU (vehicle_odometry.q 是 FRD-relative-to-NED)
        enu_orientation = CoordinateTransform.frd_ned_to_flu_enu_quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3])

        # 检查NaN值
        if any(np.isnan(enu_position)) or any(np.isnan(enu_orientation)):
            self.get_logger().warning('NaN detected in PX4 odometry conversion, skipping this message')
            self.callback_stats['px4_odom_callback']['count'] += 1
            self.callback_stats['px4_odom_callback']['total_time'] += time.time() - start_time
            return

        # 确保值是有效的float类型
        enu_position = [float(x) for x in enu_position]
        enu_orientation = [float(x) for x in enu_orientation]

        # 检查数值，如果与上一次有效位置差异过大则立刻发布切换到position模式
        if self.last_valid_enu_position is not None:
            # 计算位置差异的欧氏距离
            position_diff = (
                abs(enu_position[0] - self.last_valid_enu_position[0]) +
                abs(enu_position[1] - self.last_valid_enu_position[1]) +
                abs(enu_position[2] - self.last_valid_enu_position[2])
            )
            
            # 设置最大允许位置差异阈值（单位：米）
            MAX_POSITION_DIFF = 2.0
            
            if position_diff > MAX_POSITION_DIFF:
                self.get_logger().error(
                    f'检测到位置跳变！差异: {position_diff:.2f}m > 阈值: {MAX_POSITION_DIFF}m | '
                    f'上次ENU位置: ({self.last_valid_enu_position[0]:.2f}, {self.last_valid_enu_position[1]:.2f}, {self.last_valid_enu_position[2]:.2f}) | '
                    f'新ENU位置: ({enu_position[0]:.2f}, {enu_position[1]:.2f}, {enu_position[2]:.2f}) | '
                    f'立即切换到POSITION模式以确保安全'
                )
                # 更新上一次有效位置
                self.last_valid_enu_position = enu_position.copy()
                
                # 切换到POSITION模式 (mode 3)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 3)
                
                # 更新统计信息并返回，跳过本次处理
                self.callback_stats['px4_odom_callback']['count'] += 1
                self.callback_stats['px4_odom_callback']['total_time'] += time.time() - start_time
                return
        
        # 更新上一次有效位置
        self.last_valid_enu_position = enu_position.copy()
        

        # 发布 base_link -> px4_odom tf (相对变换)
        # PX4 odometry 表示无人机在 NED 坐标系中的位置
        # 我们需要取逆变换来表示 px4_odom 在 base_link 中的位置
        t = TransformStamped()
        # 使用当前仿真时间戳（确保与 tf_publisher 时间同步）
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.namespace.lstrip('/') + 'base_link'
        t.child_frame_id = self.namespace.lstrip('/') + 'px4_odom_body'

        # 计算逆变换：使用工具函数
        # base_link -> px4_odom 的逆变换 = base_link -> world
        inv_pos, inv_quat = CoordinateTransform.inverse_transform(
            enu_position[0], enu_position[1], enu_position[2],
            enu_orientation[0], enu_orientation[1], enu_orientation[2], enu_orientation[3]
        )

        t.transform.translation.x = float(inv_pos[0])
        t.transform.translation.y = float(inv_pos[1])
        t.transform.translation.z = float(inv_pos[2])

        t.transform.rotation.w = float(inv_quat[0])
        t.transform.rotation.x = float(inv_quat[1])
        t.transform.rotation.y = float(inv_quat[2])
        t.transform.rotation.z = float(inv_quat[3])

        # 使用正确的 sendTransform 方法
        try:
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Failed to send TF: {e}')
        
        # 读取 world->e 的 tf，并发布 world->px4_odom 的静态 tf
        try:
            # 计算 world->px4_odom 的变换
            # world->px4_odom = world->base_link * base_link->px4_odom
            # 

            try:
                # world_to_px4 = self.tf_buffer.lookup_transform(
                #     'world',
                #     self.namespace.lstrip('/') + 'px4_odom_body',
                #     rclpy.time.Time(),
                #     timeout=rclpy.duration.Duration(seconds=2.0)
                # )
                world_to_base = self.tf_buffer.lookup_transform(
                    'world',
                    base_link_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )

                world_to_px4 = self.multiply_transforms(world_to_base, t)
                # 设置静态 tf 的属性
                world_to_px4.header.stamp = self.get_clock().now().to_msg()
                world_to_px4.header.frame_id = 'world'
                world_to_px4.child_frame_id = px4_odom_frame

                # 使用 StaticTransformBroadcaster 发布静态 tf
                self.static_tf_broadcaster.sendTransform(world_to_px4)
                # self.get_logger().info(f'Published static TF: world -> {px4_odom_frame}')
            except Exception as e:
                self.get_logger().warning(f'Failed to read tf for {px4_odom_frame}: {e}')
            
            
            
        except Exception as e:
            self.get_logger().warning(f'Failed to publish static TF world->px4_odom: {e}')
        
        self.callback_stats['px4_odom_callback']['count'] += 1
        self.callback_stats['px4_odom_callback']['total_time'] += time.time() - start_time
    
    def multiply_transforms(self, t1, t2):
        """组合两个 TF 变换: result = t1 * t2"""
        result_pos, result_quat = CoordinateTransform.multiply_transforms(
            t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z,
            t1.transform.rotation.w, t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z,
            t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z,
            t2.transform.rotation.w, t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z
        )
        
        result = TransformStamped()
        result.transform.translation.x = float(result_pos[0])
        result.transform.translation.y = float(result_pos[1])
        result.transform.translation.z = float(result_pos[2])
        result.transform.rotation.w = float(result_quat[0])
        result.transform.rotation.x = float(result_quat[1])
        result.transform.rotation.y = float(result_quat[2])
        result.transform.rotation.z = float(result_quat[3])
        
        return result

    def ros2_odom_callback(self, msg: Odometry):
        """Gazebo odometry callback - 发布 PX4 visual odometry (NED坐标系)"""
        start_time = time.time()
        
        # 间隔限制检查
        current_time = time.time()
        if current_time - self.last_ros2_odom_time < self.odom_callback_min_interval:
            # self.callback_stats['ros2_odom_callback']['count'] += 1
            self.callback_stats['ros2_odom_callback']['total_time'] += time.time() - start_time
            return
        self.last_ros2_odom_time = current_time
        
        # 保存 LIO odometry 用于高度判断
        self.cur_lio_vehicle_odometry = msg
        
        # 检查 odom 消息的 frame_id，如果是 livox_frame，则转换到 base_link 坐标系
        # if self.hostname != 'ywj-B250-D3A' and self.hostname != 'DESKTOP-ypat':
        #     try:
        #         # 使用 TF 将 odom 从 livox_frame 转换到 base_link
        #         # 使用 rclpy.time.Time() 获取最新的实时变换，而不是从 buffer 中查找静态变换
        #         msg.header.frame_id = 'livox_frame'
        #         transform = self.tf_buffer.lookup_transform(
        #             'livox_frame',
        #             'base_link',
        #             rclpy.time.Time(),
        #             timeout=rclpy.duration.Duration(seconds=1.0)
        #         )
                
        #         # 转换位置
        #         odom_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
        #         msg.header.frame_id = 'world'

        #         transform = self.tf_buffer.lookup_transform(
        #             'world',
        #             'base_link',
        #             rclpy.time.Time(),
        #             timeout=rclpy.duration.Duration(seconds=1.0)
        #         )
                
        #         # 转换位置
        #         odom_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
                
        #         # 更新消息的位置和姿态
        #         msg.pose.pose = odom_transformed
        #         msg.header.frame_id = 'base_link'
                
        #         self.get_logger().debug('Transformed odom from livox_frame to base_link')
                
        #     except Exception as e:
        #         self.get_logger().warning(f'Failed to transform odom from livox_frame to base_link: {e}')
        #         # 如果转换失败，仍然使用原始消息
        
        self.publish_px4_visual_odometry(msg)  # 不发布odom，减少影响
        self.callback_stats['ros2_odom_callback']['count'] += 1
        self.callback_stats['ros2_odom_callback']['total_time'] += time.time() - start_time

    def publish_px4_visual_odometry(self, msg: Odometry):
        """转换并发布PX4 visual odometry (NED坐标系) - FLU -> NED，使用init_heading补偿"""
        try:
            px4_msg = VehicleOdometry()
            
            # 时间戳 - 转换为PX4时间基准（微秒）
            px4_msg.timestamp = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)
            px4_msg.timestamp_sample = px4_msg.timestamp
            
            # 坐标系设置
            px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            
            # 直接转换 FLU -> NED，假设PX4 NED原点与ROS2 world位置重合
            flu_x = msg.pose.pose.position.x
            flu_y = msg.pose.pose.position.y
            flu_z = msg.pose.pose.position.z
            
            # 使用标准 FLU -> NED 转换，不进行px4_odom补偿
            if self.cur_vehicle_local_position:
                px4_msg.position = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, self.init_vehicle_local_position.heading)
            
            # 使用当前PX4 odometry的方向 (已经是 FRD/NED 格式)
            if self.cur_vehicle_odometry is not None:
                px4_msg.q = self.cur_vehicle_odometry.q
            else:
                # 如果没有当前PX4 odometry，使用默认转换
                flu_qw = msg.pose.pose.orientation.w
                flu_qx = msg.pose.pose.orientation.x
                flu_qy = msg.pose.pose.orientation.y
                flu_qz = msg.pose.pose.orientation.z
                px4_msg.q = CoordinateTransform.flu_enu_to_frd_ned_quaternion(flu_qw, flu_qx, flu_qy, flu_qz)

            # 速度转换：world frame -> FRD body frame
            # Super-LIO 输出的 velocity 是世界坐标系（与位置同坐标系）
            # PX4 vehicle_odometry.velocity 必须是 FRD body frame
            # 转换链: world -> R^T -> FLU body -> FRD body
            world_vx = msg.twist.twist.linear.x
            world_vy = msg.twist.twist.linear.y
            world_vz = msg.twist.twist.linear.z
            
            flu_qw = msg.pose.pose.orientation.w
            flu_qx = msg.pose.pose.orientation.x
            flu_qy = msg.pose.pose.orientation.y
            flu_qz = msg.pose.pose.orientation.z
            R_world_to_body = CoordinateTransform.create_rotation_matrix_from_quaternion(
                flu_qw, flu_qx, flu_qy, flu_qz
            ).T
            body_vel = R_world_to_body @ np.array([world_vx, world_vy, world_vz])
            px4_msg.velocity = CoordinateTransform.flu_to_frd_velocity(
                float(body_vel[0]), float(body_vel[1]), float(body_vel[2])
            )

            # 角速度转换：FLU -> FRD
            # ROS: angular velocity -> body frame (FLU)
            # PX4: angular velocity -> body frame (FRD)
            # 只需要坐标轴反射: wx=wx, wy=-wy, wz=-wz
            flu_wx = msg.twist.twist.angular.x
            flu_wy = msg.twist.twist.angular.y
            flu_wz = msg.twist.twist.angular.z
            px4_msg.angular_velocity = CoordinateTransform.flu_to_frd_angular_velocity(flu_wx, flu_wy, flu_wz)
            
            # 协方差 (简化处理)
            px4_msg.position_variance = [0.0001, 0.0001, 0.0001]
            px4_msg.orientation_variance = [0.01, 0.01, 0.01]
            px4_msg.velocity_variance = [0.01, 0.01, 0.01]
            
            # 质量指标
            px4_msg.quality = 100
            px4_msg.reset_counter = 0
            
            # 发布消息
            self.px4_visual_pub.publish(px4_msg)
            
            self.get_logger().debug(
                f'PX4 Visual Odom: pos=[{px4_msg.position[0]:.2f}, {px4_msg.position[1]:.2f}, {px4_msg.position[2]:.2f}]',
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting to PX4 visual odometry: {str(e)}')

    def goal_marker_callback(self, msg):
        """处理目标点标记，触发自动状态切换"""
        start_time = time.time()
        self.last_goal_marker_time = self.get_clock().now()
        
        # 只有在自动切换未完成或无人机已落地的情况下才重新启动切换流程
        if not self.auto_switch_enabled and not self.auto_switch_completed:
            self.get_logger().info('收到目标点标记，开始自动状态切换流程')
            self.auto_switch_enabled = True
            self.auto_switch_completed = False  # 重置完成标志
            self.auto_switch_to_egoplanner()
        elif self.auto_switch_completed and not self.auto_switch_enabled:
            # 检查无人机是否可以重新启动切换
            if self.vehicle_status is not None:
                current_altitude = self.get_current_altitude()
                is_armed = self.vehicle_status.arming_state == 2
                nav_state = self.vehicle_status.nav_state
                
                # 检查是否可以重新启动切换：
                # 1. 已落地且未解锁
                # 2. 或者在降落状态（nav_state=18）
                if (current_altitude <= 0.3 and not is_armed) or nav_state == 18:
                    self.get_logger().info(f'收到新目标点标记，无人机状态: 高度={current_altitude:.2f}m, 已解锁={is_armed}, nav_state={nav_state}，重新启动自动切换流程')
                    self.auto_switch_enabled = True
                    self.auto_switch_completed = False
                    self.auto_switch_to_egoplanner()
                else:
                    self.get_logger().debug('收到目标点标记，但无人机仍在飞行状态，不做状态变换')
            else:
                self.get_logger().warn('收到目标点标记，但无法获取无人机状态，不做状态变换')
        self.callback_stats['goal_marker_callback']['count'] += 1
        self.callback_stats['goal_marker_callback']['total_time'] += time.time() - start_time
    
    def get_clock_microseconds(self):
        t_ = self.get_clock().now().seconds_nanoseconds()
        return int(t_[0]*1e6 + t_[1]/1000)

    def cmd_pose_local_ned_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_pose_local_ned_callback']['count'] += 1
            self.callback_stats['cmd_pose_local_ned_callback']['total_time'] += time.time() - start_time
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_NED"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [ orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = quat2euler(orientation_list)
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [msg.position.x, msg.position.y, msg.position.z]
        cmd.velocity = [math.nan, math.nan, math.nan]
        cmd.acceleration = [math.nan, math.nan, math.nan]
        cmd.yaw = yaw
        cmd.yawspeed = math.nan
        self.cmd = cmd
        self.callback_stats['cmd_pose_local_ned_callback']['count'] += 1
        self.callback_stats['cmd_pose_local_ned_callback']['total_time'] += time.time() - start_time
        
    def cmd_pose_local_flu_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_pose_local_flu_callback']['count'] += 1
            self.callback_stats['cmd_pose_local_flu_callback']['total_time'] += time.time() - start_time
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_FLU"
        # Convert quaternion to euler angles
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = quat2euler(orientation_list)

        # 使用 TF 变换将 FLU -> NED
        # FLU 坐标系：机体坐标系
        # NED 坐标系：px4_odom再变换坐标系（PX4）
        # 通过 TF 变换：world -> px4_odom
        try:
            px4_odom_frame = self.namespace.lstrip('/') + 'px4_odom'
            world_frame = 'world'
            
            # 直接使用接收到的 PoseStamped 进行 TF 变换
            pose_stamped = msg
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # 使用 TF 变换将位姿从  world -> px4_odom
            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                px4_odom_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # ENU -> NED 坐标转换 (使用工具类)
            p_n, p_e, p_d = CoordinateTransform.enu_to_ned_position(
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z
            )
            
            # 先将 ENU 四元数转换为 NED 四元数
            ned_qw, ned_qx, ned_qy, ned_qz = CoordinateTransform.enu_to_ned_quaternion(
                transformed_pose.pose.orientation.w,
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z
            )
            
            # 航向角：从 NED 四元数提取
            transformed_yaw = CoordinateTransform.heading_from_quaternion(
                ned_qw, ned_qx, ned_qy, ned_qz
            )
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [p_n, p_e, p_d]
            # transformed_yaw 已经是 NED 坐标系下的航向角，不需要额外补偿
            cmd.velocity = [math.nan, math.nan, math.nan]
            cmd.acceleration = [math.nan, math.nan, math.nan]
            cmd.yaw = transformed_yaw
            cmd.yawspeed = math.nan
            self.cmd = cmd
            
        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_pose_local_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时，使用当前航向角（备用方案）
            # theta = self.cur_vehicle_local_position.heading if self.cur_vehicle_local_position else 0.0
            
            # # Transfer position from FLU to NED, msg.position.xyz is flu, respectively
            # p_n = msg.position.x * math.cos(theta) + msg.position.y * math.sin(theta)
            # p_e = msg.position.x * math.sin(theta) - msg.position.y * math.cos(theta)
            # p_d = -msg.position.z
            
            # # Construct TrajectorySetpoint message
            # cmd = TrajectorySetpoint()
            # cmd.timestamp = self.get_clock_microseconds()
            # cmd.position = [p_n, p_e, p_d]
            # cmd.yaw = self.init_vehicle_local_position.heading + yaw if self.init_vehicle_local_position else yaw
            # self.cmd = cmd
        self.callback_stats['cmd_pose_local_flu_callback']['count'] += 1
        self.callback_stats['cmd_pose_local_flu_callback']['total_time'] += time.time() - start_time

    def cmd_trajectory_flu_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_trajectory_flu_callback']['count'] += 1
            self.callback_stats['cmd_trajectory_flu_callback']['total_time'] += time.time() - start_time
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_FLU"
        
        flu_x = msg.position.x
        flu_y = msg.position.y
        flu_z = msg.position.z
        flu_vx = msg.velocity.x
        flu_vy = msg.velocity.y
        flu_vz = msg.velocity.z
        flu_ax = msg.acceleration.x
        flu_ay = msg.acceleration.y
        flu_az = msg.acceleration.z
        flu_yaw = msg.yaw
        
        try:
            px4_odom_frame = self.namespace.lstrip('/') + 'px4_odom'
            
            world_to_px4odom = self.tf_buffer.lookup_transform(
                px4_odom_frame,
                'world',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            q_tf = world_to_px4odom.transform.rotation
            t_tf = world_to_px4odom.transform.translation
            R = CoordinateTransform.create_rotation_matrix_from_quaternion(
                q_tf.w, q_tf.x, q_tf.y, q_tf.z
            )
            
            pos_world = np.array([flu_x, flu_y, flu_z])
            pos_enu = R @ pos_world + np.array([t_tf.x, t_tf.y, t_tf.z])
            p_n, p_e, p_d = CoordinateTransform.enu_to_ned_position(
                float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2])
            )
            
            half_yaw = flu_yaw / 2.0
            yaw_quat_w = math.cos(half_yaw)
            yaw_quat_x = 0.0
            yaw_quat_y = 0.0
            yaw_quat_z = math.sin(half_yaw)
            
            transformed_quat = CoordinateTransform.qmult(
                q_tf.w, q_tf.x, q_tf.y, q_tf.z,
                yaw_quat_w, yaw_quat_x, yaw_quat_y, yaw_quat_z
            )
            
            ned_qw, ned_qx, ned_qy, ned_qz = CoordinateTransform.enu_to_ned_quaternion(
                transformed_quat[0], transformed_quat[1], transformed_quat[2], transformed_quat[3]
            )
            
            transformed_yaw = CoordinateTransform.heading_from_quaternion(
                ned_qw, ned_qx, ned_qy, ned_qz
            )
            
            vel_world = np.array([flu_vx, flu_vy, flu_vz])
            vel_enu = R @ vel_world
            v_n, v_e, v_d = CoordinateTransform.enu_to_ned_velocity(
                float(vel_enu[0]), float(vel_enu[1]), float(vel_enu[2])
            )
            
            acc_world = np.array([flu_ax, flu_ay, flu_az])
            acc_enu = R @ acc_world
            a_n, a_e, a_d = CoordinateTransform.enu_to_ned_acceleration(
                float(acc_enu[0]), float(acc_enu[1]), float(acc_enu[2])
            )
            
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [p_n, p_e, p_d]
            cmd.velocity = [v_n, v_e, v_d]
            cmd.acceleration = [a_n, a_e, a_d]
            cmd.yaw = transformed_yaw
            cmd.yawspeed = math.nan
            self.cmd = cmd
            
        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_trajectory_flu: {str(tf_error)}')
        
        self.callback_stats['cmd_trajectory_flu_callback']['count'] += 1
        self.callback_stats['cmd_trajectory_flu_callback']['total_time'] += time.time() - start_time

    def cmd_vel_ned_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_vel_ned_callback']['count'] += 1
            self.callback_stats['cmd_vel_ned_callback']['total_time'] += time.time() - start_time
            return
        
        self.OFFBOARD_STATE = "VEL_NED"
        
        # 从 PoseStamped 中提取位置和姿态
        # 注意：这里虽然函数名是 cmd_vel_ned，但实际处理的是 PoseStamped（位置+姿态）
        # 可能是历史遗留命名问题
        
        # Convert quaternion to euler angles
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = quat2euler(orientation_list)
        
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        cmd.velocity = [math.nan, math.nan, math.nan]
        cmd.acceleration = [math.nan, math.nan, math.nan]
        cmd.yaw = yaw
        cmd.yawspeed = math.nan
        self.cmd = cmd
        self.callback_stats['cmd_vel_ned_callback']['count'] += 1
        self.callback_stats['cmd_vel_ned_callback']['total_time'] += time.time() - start_time

    def cmd_vel_flu_callback(self, msg):
        """FLU速度 -> NED速度 (使用TF或heading)"""
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_vel_flu_callback']['count'] += 1
            self.callback_stats['cmd_vel_flu_callback']['total_time'] += time.time() - start_time
            return

        self.OFFBOARD_STATE = "VEL_FLU"

        # 使用 TF 变换将 FLU -> NED
        try:
            base_link_frame = self.namespace.lstrip('/') + 'base_link'
            world_frame = 'world'

            # 获取 world -> base_link 的 TF 变换
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                base_link_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 从 TF 变换中提取旋转矩阵
            # transform 是 world(ENU) -> base_link(FLU) 的变换
            # 我们需要 FLU -> ENU 的旋转矩阵，所以取逆
            q = transform.transform.rotation
            # 四元数共轭得到逆旋转 (ENU -> FLU 的逆 = FLU -> ENU)
            qw_inv, qx_inv, qy_inv, qz_inv = q.w, -q.x, -q.y, -q.z
            rotation_matrix_flu_to_enu = CoordinateTransform.create_rotation_matrix_from_quaternion(
                qw_inv, qx_inv, qy_inv, qz_inv)

            # 使用工具函数将 FLU 速度转换到 NED
            # 正确流程: FLU body -> FLU->ENU -> ENU->NED -> NED world
            flu_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            ned_vel = CoordinateTransform.flu_to_ned_vector_by_rotation_matrix(
                flu_vel, rotation_matrix_flu_to_enu)

            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [float(ned_vel[0]), float(ned_vel[1]), float(ned_vel[2])]
            cmd.yaw = math.nan
            cmd.yawspeed = CoordinateTransform.flu_to_ned_yawspeed(msg.angular.z)
            self.cmd = cmd

        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_vel_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时的处理已在注释中，如需使用heading方式可取消注释
        self.callback_stats['cmd_vel_flu_callback']['count'] += 1
        self.callback_stats['cmd_vel_flu_callback']['total_time'] += time.time() - start_time
        
    def cmd_accel_ned_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_accel_ned_callback']['count'] += 1
            self.callback_stats['cmd_accel_ned_callback']['total_time'] += time.time() - start_time
            return

        self.OFFBOARD_STATE = "ACCEL_NED"
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [math.nan, math.nan, math.nan]
        cmd.velocity = [math.nan, math.nan, math.nan]
        cmd.acceleration = [msg.linear.x, msg.linear.y, msg.linear.z]
        #TODO: How about yaw?
        self.cmd = cmd
        self.callback_stats['cmd_accel_ned_callback']['count'] += 1
        self.callback_stats['cmd_accel_ned_callback']['total_time'] += time.time() - start_time
        
    def cmd_accel_flu_callback(self, msg):
        """FLU加速度 -> NED加速度 (使用TF)"""
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_accel_flu_callback']['count'] += 1
            self.callback_stats['cmd_accel_flu_callback']['total_time'] += time.time() - start_time
            return

        self.OFFBOARD_STATE = "ACCEL_FLU"

        # 使用 TF 变换将 FLU -> NED
        try:
            base_link_frame = self.namespace.lstrip('/') + 'base_link'
            world_frame = 'world'

            # 获取 world -> base_link 的 TF 变换
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                base_link_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 从 TF 变换中提取旋转矩阵
            # transform 是 world(ENU) -> base_link(FLU) 的变换
            # 我们需要 FLU -> ENU 的旋转矩阵，所以取逆
            q = transform.transform.rotation
            # 四元数共轭得到逆旋转 (ENU -> FLU 的逆 = FLU -> ENU)
            qw_inv, qx_inv, qy_inv, qz_inv = q.w, -q.x, -q.y, -q.z
            rotation_matrix_flu_to_enu = CoordinateTransform.create_rotation_matrix_from_quaternion(
                qw_inv, qx_inv, qy_inv, qz_inv)

            # 使用工具函数将 FLU 加速度转换到 NED
            # 正确流程: FLU body -> FLU->ENU -> ENU->NED -> NED world
            flu_accel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            ned_accel = CoordinateTransform.flu_to_ned_vector_by_rotation_matrix(
                flu_accel, rotation_matrix_flu_to_enu)

            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [math.nan, math.nan, math.nan]
            cmd.acceleration = [float(ned_accel[0]), float(ned_accel[1]), float(ned_accel[2])]
            self.cmd = cmd

        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_accel_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时的处理已在注释中，如需使用heading方式可取消注释
        self.callback_stats['cmd_accel_flu_callback']['count'] += 1
        self.callback_stats['cmd_accel_flu_callback']['total_time'] += time.time() - start_time
    
    def cmd_attitude_flu_callback(self, msg):
        start_time = time.time()
        if self.OFFBOARD_STATE == "DISABLED":
            self.callback_stats['cmd_attitude_flu_callback']['count'] += 1
            self.callback_stats['cmd_attitude_flu_callback']['total_time'] += time.time() - start_time
            return
        
        self.OFFBOARD_STATE = "ATTITUDE_FLU"
        cmd = VehicleAttitudeSetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.q_d = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        cmd.thrust = msg.linear.x
        self.cmd = cmd
        self.callback_stats['cmd_attitude_flu_callback']['count'] += 1
        self.callback_stats['cmd_attitude_flu_callback']['total_time'] += time.time() - start_time


    def cmd_callback(self, request, response):
        command = request.command
        if command == "ARM":
            self.arm()
            response.success = True
        elif command == "DISARM":
            self.disarm()
            response.success = True
        elif command == "HOVER":
            self.hover()
            response.success = True
        elif command == "OFFBOARD":
            self.OFFBOARD_STATE = "ENABLED"
            self.get_logger().info('Offboard command send')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            response.success = True
        elif command == "TAKEOFF":
            self.takeoff()
            response.success = True
        elif command == "LAND":
            self.land()
            response.success = True
        elif command == "RTL":
            self.rtl()
            response.success = True
        else:
            self.get_logger().warn(f'Unknown command: {command}')       
            response.success = False 
        return response

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock_microseconds()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        # TODO：Check if the vehicle is already armed
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command send')
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command send')
    
    def hover(self):
        # self.OFFBOARD_STATE = "DISABLED"
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_PAUSE_CONTINUE, 0.0)
        self.get_logger().info('Hover command send')

    def takeoff(self):
        # | Minimum pitch (if airspeed sensor present), desired pitch without sensor
        # | Empty
        # | Empty
        # | Yaw angle (if magnetometer present), ignored without magnetometer
        # | Latitude
        # | Longitude
        # | Altitude (限制为1.5米)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_local_position.ref_lat, param6=self.cur_vehicle_local_position.ref_lon, param7=1.5)
        self.get_logger().info("Take off command send. Target altitude: 1.5m")

    def land(self):
        # | Empty
        # | Empty
        # | Empty
        # | Desired yaw angle.
        # | Latitude
        # | Longitude
        # | Altitude|
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_global_position.lat, param6=self.cur_vehicle_global_position.lon, param7=0.0)
        self.get_logger().info("Land command send.")
    
    def rtl(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command send.")

    def publish_vehicle_state(self):
        if not self.debug:
            return
            
        msg = XTD2VehicleState()
        msg.timestamp = self.get_clock_microseconds()
        msg.offboard_state = self.OFFBOARD_STATE

        # 当前位置信息
        if self.cur_vehicle_local_position is not None:
            msg.x = self.cur_vehicle_local_position.x
            msg.y = self.cur_vehicle_local_position.y
            msg.z = self.cur_vehicle_local_position.z
            msg.heading = self.cur_vehicle_local_position.heading
        else:
            msg.x = float('nan')
            msg.y = float('nan')
            msg.z = float('nan')
            msg.heading = float('nan')

        # 全局位置信息
        if self.cur_vehicle_global_position is not None:
            msg.lat = self.cur_vehicle_global_position.lat
            msg.lon = self.cur_vehicle_global_position.lon
            msg.alt = self.cur_vehicle_global_position.alt
        else:
            msg.lat = float('nan')
            msg.lon = float('nan')
            msg.alt = float('nan')

        # 初始位置信息
        if self.init_vehicle_local_position is not None:
            msg.init_x = self.init_vehicle_local_position.x
            msg.init_y = self.init_vehicle_local_position.y
            msg.init_z = self.init_vehicle_local_position.z
            msg.init_heading = self.init_vehicle_local_position.heading
        else:
            msg.init_x = float('nan')
            msg.init_y = float('nan')
            msg.init_z = float('nan')
            msg.init_heading = float('nan')

        # 初始全局位置信息
        if self.init_vehicle_global_position is not None:
            msg.init_lat = self.init_vehicle_global_position.lat
            msg.init_lon = self.init_vehicle_global_position.lon
            msg.init_alt = self.init_vehicle_global_position.alt
        else:
            msg.init_lat = float('nan')
            msg.init_lon = float('nan')
            msg.init_alt = float('nan')

        self.vehicle_state_publisher.publish(msg)

    def get_current_altitude(self):
        """获取当前高度，优先使用 LIO odometry，否则使用 PX4 local position"""
        if self.cur_lio_vehicle_odometry is not None:
            # LIO odometry 使用 ENU 坐标系，z 向上为正
            return self.cur_lio_vehicle_odometry.pose.pose.position.z
        elif self.cur_vehicle_local_position is not None:
            # PX4 local position 使用 NED 坐标系，z 向下为负
            return -self.cur_vehicle_local_position.z
        else:
            return 0.0

    def auto_switch_to_egoplanner(self):
        """自动切换到egoplanner控制模式"""
        if self.vehicle_status is None:
            self.get_logger().warn('无法获取无人机状态，等待数据...')
            return
        
        # 1. 检查是否已起飞（使用高度判断，优先使用 LIO odometry）
        current_altitude = self.get_current_altitude()
        is_flying = current_altitude > 0.3  # 高度超过0.3米认为已起飞
        
        # 2. 检查是否已解锁
        is_armed = self.vehicle_status.arming_state == 2  # ARMING_STATE_ARMED
        
        # 3. 检查是否在offboard模式
        is_offboard = self.vehicle_status.nav_state == 14  # NAVIGATION_STATE_OFFBOARD

        # 4. 检查是否在起飞模式
        is_takeoff = self.vehicle_status.nav_state == 17  # NAVIGATION_STATE_TAKEOFF

        is_landing = self.vehicle_status.nav_state == 18  # NAVIGATION_STATE_LAND
        
        self.get_logger().info(f'状态检查: nav_state={self.vehicle_status.nav_state}, 高度={current_altitude:.2f}m, 已起飞={is_flying}, 已解锁={is_armed}, offboard模式={is_offboard}')
        
        # 执行状态切换（按照PX4安全要求：先解锁，再起飞，最后切换到offboard模式）
        # 只有在非offboard模式时才切换
        
        
        # 已经在offboard模式，检查是否需要解锁
        if not is_armed:
            self.get_logger().info('无人机未解锁，执行解锁...')
            self.arm()
            self.get_logger().info('等待解锁完成...')
            return
        
        # 已解锁，检查是否需要起飞
        if not is_takeoff and not is_flying and not is_landing:
            self.get_logger().info('无人机未起飞，执行起飞...')
            self.takeoff()
            self.get_logger().info('等待起飞完成...')
            return
        
        if not is_offboard and not is_landing and is_flying:
            self.get_logger().info('已起飞，最后切换到offboard模式...')
            self.OFFBOARD_STATE = "ENABLED"
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.get_logger().info('offboard模式切换完成')
            return
        
        # 所有条件满足，切换到egoplanner控制
        if is_offboard:
            self.get_logger().info('无人机已准备就绪，可以接收egoplanner控制指令')
            self.auto_switch_enabled = False  # 重置标志
            self.auto_switch_completed = True  # 设置完成标志

    def check_auto_switch_progress(self):
        """检查自动切换进度，确保状态转换完成"""
        if self.vehicle_status is None:
            return
        
        current_altitude = self.get_current_altitude()
        is_armed = self.vehicle_status.arming_state == 2
        is_offboard = self.vehicle_status.nav_state == 14
        is_flying = current_altitude > 0.3  # 使用高度判断是否在飞行状态
        
        # 如果所有条件都满足，完成切换
        if is_armed and is_flying and is_offboard:
            self.get_logger().info('自动切换完成：无人机已解锁、起飞并进入offboard模式')
            self.auto_switch_enabled = False
            self.auto_switch_completed = True
            return
        
        # 如果超过30秒仍未完成切换，重置状态
        if self.last_goal_marker_time is not None:
            elapsed_time = (self.get_clock().now() - self.last_goal_marker_time).nanoseconds / 1e9
            if elapsed_time > 30:
                self.get_logger().warn('自动切换超时，重置状态')
                self.auto_switch_enabled = False
                return
        
        # 每1秒重新检查一次状态
        if hasattr(self, '_last_check_time'):
            elapsed = (self.get_clock().now() - self._last_check_time).nanoseconds / 1e9
            if elapsed < 1:
                return
        
        self._last_check_time = self.get_clock().now()
        self.get_logger().info(f'自动切换进度: 已解锁={is_armed}, 已起飞={is_flying}, offboard模式={is_offboard}')
        
        # 重新执行切换逻辑
        self.auto_switch_to_egoplanner()

    def check_landing_and_disarm(self):
        """检查无人机是否落地，并在确认落地后自动解除arm"""
        if self.vehicle_status is None:
            return
        
        # 获取当前高度（优先使用 LIO odometry）
        current_altitude = self.get_current_altitude()
        is_armed = self.vehicle_status.arming_state == 2
        nav_state = self.vehicle_status.nav_state
        
        # 使用PX4 nav_state判断飞行状态
        # nav_state=14表示offboard模式（飞行状态）
        # nav_state=18表示降落状态
        # 使用高度和nav_state综合判断是否在飞行状态
        is_offboard = nav_state == 14
        is_landing = nav_state == 18
        is_flying = is_offboard and current_altitude > 0.3  # 在offboard模式且高度超过0.3米认为在飞行
        
        # 检测状态变化：从飞行状态变为降落状态
        if self.was_flying and is_landing and is_armed:
            # 首次检测到降落，先切换到hold模式，再切换回降落模式
            
            if self.landed_time is None:
                # self.get_logger().info(f'检测到无人机降落，nav_state={nav_state}, 高度={current_altitude:.2f}m，切换到hold模式...')
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4)
                # time.sleep(2)
                # self.get_logger().info(f'检测到无人机降落，nav_state={nav_state}，切换回降落模式...')
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 21)
                self.landed_time = self.get_clock().now()
            else:
                # 检查降落确认时间（3秒）
                elapsed_time = (self.get_clock().now() - self.landed_time).nanoseconds / 1e9
                if elapsed_time >= 3.0:
                    self.get_logger().info(f'检测到无人机降落，nav_state={nav_state}，切换降落模式...')
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 21)
                    self.get_logger().info('等待确认无人机降落，发送解除arm...')
                    self.disarm()
                    # self.landed_time = None
        elif is_flying:
            # 无人机在飞行状态，重置降落计时器
            self.landed_time = None
            self.was_flying = True

        elif self.was_flying and not is_armed:
            # 在降落状态且已解除arm，也重置状态
            self.landed_time = None
            self.was_flying = False
            # 重置自动切换完成标志，允许下次收到goal marker时重新启动
            self.get_logger().info(f'无人机降落完成，nav_state={nav_state}, 高度={current_altitude:.2f}m，切换到position模式...')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 3)
            if self.auto_switch_completed:
                self.get_logger().info(f'无人机在降落状态(nav_state={nav_state})且已解除arm，重置自动切换状态，等待新目标点')
                self.auto_switch_completed = False
                self.auto_switch_enabled = False
        
        # 更新飞行状态记录
        if is_flying:
            self.was_flying = is_flying
    
    def trigger_landing(self):
        """触发无人机降落"""
        if self.vehicle_status is None:
            return
        
        is_armed = self.vehicle_status.arming_state == 2
        nav_state = self.vehicle_status.nav_state
        
        if not is_armed:
            self.get_logger().warn('无人机未解锁，无法降落')
            return
        
        if nav_state == 18:
            self.get_logger().info('无人机已在降落状态')
            return
        
        self.get_logger().info(f'触发降落，当前nav_state={nav_state}')
        
        # 发送LAND命令
        self.land()
        
        # 关闭offboard心跳
        self.OFFBOARD_STATE = "DISABLED"
        self.get_logger().info('已关闭offboard模式，进入降落')


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif value.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError(f'Boolean value expected, got {value}')


def main():
    rclpy.init(args=sys.argv)
    
    parser = argparse.ArgumentParser(description='XTDrone2 Multirotor Communication Node')

    parser.add_argument('--model', type=str, help='Vehicle type', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id, should be unique in same model', required=True)
    parser.add_argument('--allowarm', type=str_to_bool, help='Allow arm command', required=False, default=False)
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")
    parser.add_argument('--debug', action='store_true', help='Enable debug mode to publish vehicle state', required=False, default=False)
    

    args, unknown = parser.parse_known_args()

    multirotor_communication = MultirotorCommunication(args.model, args.id, args.namespace, args.debug, args.allowarm)
    rclpy.spin(multirotor_communication)
    multirotor_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()