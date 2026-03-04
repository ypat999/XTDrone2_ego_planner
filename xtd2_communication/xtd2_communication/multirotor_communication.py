"""
Multirotor Communication Module

This module provides communication functionalities for multirotor.

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

All rights reserved. This code is licensed under the MIT License.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleAttitudeSetpoint, VehicleStatus, VehicleOdometry
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

class MultirotorCommunication(Node):
    def __init__(self, model, id, namespace="", debug=False):
        
        if model.startswith("gz_"):  # 删除gz_前缀
            model = model[3:]
        self.model = model

        self.id = int(id)
        self.debug = debug

        self.namespace = namespace if namespace else f'{model}_{id}'

        # 生成有效的节点名称（只包含字母数字和下划线）
        node_name = self.namespace.strip('/') + '_communication'
        
        # 检查主机名，设置 use_sim_time（与 tf_publisher 保持一致）
        import platform
        hostname = platform.node()
        if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
            use_sim_time = True
        else:
            use_sim_time = False
            
        super().__init__(node_name, parameter_overrides=[
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        self.OFFBOARD_STATE = "DISABLED"
        self.cmd = None
        self.cur_vehicle_local_position = None
        self.cur_vehicle_global_position = None
        self.init_vehicle_local_position = None
        self.init_vehicle_global_position = None
        self.vehicle_status = None
        self.auto_switch_enabled = False
        self.last_goal_marker_time = None
        self.was_flying = False  # 记录之前是否在飞行状态
        self.landed_time = None  # 记录落地时间
        self.auto_switch_completed = False  # 记录自动切换是否已完成

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
        self.create_subscription(Pose, xtdrone2_topic_prefix + 'cmd_pose_local_flu', self.cmd_pose_local_flu_callback, 10)  # geometry_msgs/Pose
        self.create_subscription(Twist, xtdrone2_topic_prefix + 'cmd_vel_ned', self.cmd_vel_ned_callback, 10)  # geometry_msgs/Twist
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
        self.static_tf_published = False  # 标记静态 tf 是否已发布
        
        # TF buffer and listener for coordinate transformations
        # 优化 TF 缓冲区配置，提高实时性
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))  # 减少缓存时间到2秒
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)  # 启用独立线程
        
        # Goal marker subscription for automatic switching
        self.create_subscription(PoseStamped, '/goal_pose_3d', self.goal_marker_callback, 10)
        
        # Gazebo odometry subscription for PX4 visual odometry
        self.create_subscription(
            Odometry,
            self.namespace + 'odometry',
            self.gazebo_odom_callback,
            10
        )

        self.timer_ = self.create_timer(0.05, self.timer_callback)

        # Debug publisher for vehicle state
        if self.debug:
            self.vehicle_state_publisher = self.create_publisher(XTD2VehicleState, xtdrone2_topic_prefix + 'debug/vehicle_state', 10)
            self.debug_timer = self.create_timer(0.1, self.publish_vehicle_state)  # 10Hz

        self.get_logger().info(f'{self.namespace} communication node started')
    
    def timer_callback(self):
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
            return
        
        if self.OFFBOARD_STATE == "DISABLED":
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
    
    def vehicle_local_position_callback(self, msg):
        if self.init_vehicle_local_position is None:
            self.init_vehicle_local_position = msg
        self.cur_vehicle_local_position = msg

    def vehicle_global_position_callback(self, msg):
        if self.init_vehicle_global_position is None:
            self.init_vehicle_global_position = msg
        self.cur_vehicle_global_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def px4_odom_callback(self, msg):
        """PX4 odometry callback - 发布 base_footprint -> px4_odom tf (相对变换)"""
        
        # PX4 odometry 表示无人机在 NED 坐标系中的位置
        # 我们需要发布 base_footprint -> px4_odom 的相对变换
        # 这个变换表示 px4_odom 在 base_footprint 坐标系中的位置

        # 读取 world->base_footprint 的 tf
        base_footprint_frame = self.namespace.lstrip('/') + 'base_footprint'
        px4_odom_frame = self.namespace.lstrip('/') + 'px4_odom'
        try:
            world_to_base = self.tf_buffer.lookup_transform(
                'world',
                base_footprint_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warning(f'Failed to read tf')

        # 转换坐标系：NED -> ENU
        enu_position = self.ned_to_enu_position(msg.position[0], msg.position[1], msg.position[2])
        enu_orientation = self.ned_to_enu_quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        
        # 检查NaN值
        if any(np.isnan(enu_position)) or any(np.isnan(enu_orientation)):
            self.get_logger().warning('NaN detected in PX4 odometry conversion, skipping this message')
            return
        
        # 确保值是有效的float类型
        enu_position = [float(x) for x in enu_position]
        enu_orientation = [float(x) for x in enu_orientation]
        
        # 发布 base_footprint -> px4_odom tf (相对变换)
        # PX4 odometry 表示无人机在 NED 坐标系中的位置
        # 我们需要取逆变换来表示 px4_odom 在 base_footprint 中的位置
        t = TransformStamped()
        # 使用当前仿真时间戳（确保与 tf_publisher 时间同步）
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
        t.child_frame_id = self.namespace.lstrip('/') + 'px4_odom'
        
        # 正确的逆变换：考虑90度右转调整
        # 首先计算逆旋转矩阵（四元数共轭对应的旋转矩阵）
        qw, qx, qy, qz = enu_orientation[0], -enu_orientation[1], -enu_orientation[2], -enu_orientation[3]
        
        # 计算逆旋转矩阵
        R_inv = np.array([
            [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]
        ])
        
        # 90度右转的旋转矩阵 (绕Z轴旋转-90度)
        yaw_90_right = -np.pi/2  # -90度
        R_yaw = np.array([
            [np.cos(yaw_90_right), -np.sin(yaw_90_right), 0],
            [np.sin(yaw_90_right), np.cos(yaw_90_right), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转矩阵：先逆旋转，再右转90度
        R_combined = R_yaw @ R_inv
        
        # 将位置向量变换到0方向
        position_vector = np.array([enu_position[0], enu_position[1], enu_position[2]])
        transformed_position = R_combined @ (-position_vector)
        
        t.transform.translation.x = float(transformed_position[0])
        t.transform.translation.y = float(transformed_position[1])
        t.transform.translation.z = float(transformed_position[2])
        
        # 计算组合四元数：先逆变换，再右转90度
        # 将逆旋转矩阵转换为四元数
        from transforms3d.quaternions import mat2quat
        combined_quat = mat2quat(R_combined)
        
        t.transform.rotation.w = float(combined_quat[0])
        t.transform.rotation.x = float(combined_quat[1])
        t.transform.rotation.y = float(combined_quat[2])
        t.transform.rotation.z = float(combined_quat[3])

        # 使用正确的 sendTransform 方法
        # try:
        #     self.tf_broadcaster.sendTransform(t)
        # except Exception as e:
        #     self.get_logger().error(f'Failed to send TF: {e}')
        
        # 读取 world->base_footprint 的 tf，并发布 world->px4_odom 的静态 tf
        # if not self.static_tf_published:
        try:
            # 计算 world->px4_odom 的变换
            # world->px4_odom = world->base_footprint * base_footprint->px4_odom
            world_to_px4 = self.multiply_transforms(world_to_base, t)
            
            # 设置静态 tf 的属性
            world_to_px4.header.stamp = self.get_clock().now().to_msg()
            world_to_px4.header.frame_id = 'world'
            world_to_px4.child_frame_id = px4_odom_frame
            
            # 使用 StaticTransformBroadcaster 发布静态 tf
            self.static_tf_broadcaster.sendTransform(world_to_px4)
            self.static_tf_published = True
            # self.get_logger().info(f'Published static TF: world -> {px4_odom_frame}')
            
        except Exception as e:
            self.get_logger().warning(f'Failed to publish static TF world->px4_odom: {e}')
    
    def multiply_transforms(self, t1, t2):
        """组合两个 TF 变换: result = t1 * t2"""
        from transforms3d.quaternions import qmult, quat2mat, mat2quat
        
        # 提取 t1 的平移和旋转
        t1_trans = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
        t1_quat = np.array([t1.transform.rotation.w, t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z])
        
        # 提取 t2 的平移和旋转
        t2_trans = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])
        t2_quat = np.array([t2.transform.rotation.w, t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z])
        
        # 组合旋转四元数
        result_quat = qmult(t1_quat, t2_quat)
        
        # 组合平移向量: result_trans = t1_trans + R1 * t2_trans
        R1 = quat2mat(t1_quat)
        result_trans = t1_trans + R1 @ t2_trans
        
        # 创建结果变换
        result = TransformStamped()
        result.transform.translation.x = float(result_trans[0])
        result.transform.translation.y = float(result_trans[1])
        result.transform.translation.z = float(result_trans[2])
        result.transform.rotation.w = float(result_quat[0])
        result.transform.rotation.x = float(result_quat[1])
        result.transform.rotation.y = float(result_quat[2])
        result.transform.rotation.z = float(result_quat[3])
        
        return result

    def ned_to_enu_position(self, x, y, z):
        """将NED坐标系的位置转换为ENU坐标系"""
        # NED: x=North, y=East, z=Down
        # ENU: x=East, y=North, z=Up
        return [y, x, -z]

    def ned_to_enu_quaternion(self, qw, qx, qy, qz):
        """将NED坐标系的四元数转换为ENU坐标系"""
        R_ned = self.quat_to_rot(qw, qx, qy, qz)

        T = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # T_yaw = np.array([
        #     [0, -1, 0],
        #     [1, 0, 0],
        #     [0, 0, 1]
        # ])

        # r_enu_final = T_yaw.T @ (T @ R_ned @ T.T)
        r_enu_final =  T @ R_ned @ T.T

        return self.rot_to_quat(r_enu_final)

    def quat_to_rot(self, w, x, y, z):
        return np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
        ])

    def rot_to_quat(self, R):
        """将旋转矩阵转换为四元数（使用更健壮的算法）"""
        # 使用Shepperd方法，避免数值不稳定
        tr = np.trace(R)
        
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        
        return [float(w), float(x), float(y), float(z)]

    def gazebo_odom_callback(self, msg: Odometry):
        """Gazebo odometry callback - 发布 PX4 visual odometry (NED坐标系)"""
        self.publish_px4_visual_odometry(msg)

    def publish_px4_visual_odometry(self, msg: Odometry):
        """转换并发布PX4 visual odometry (NED坐标系) - 直接转换 FLU -> NED"""
        try:
            px4_msg = VehicleOdometry()
            
            # 时间戳 - 转换为PX4时间基准（微秒）
            px4_msg.timestamp = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)
            px4_msg.timestamp_sample = px4_msg.timestamp
            
            # 坐标系设置
            px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            
            # 直接转换 FLU -> NED
            # FLU: x=前, y=左, z=上
            # NED: x=北, y=东, z=下
            flu_x = msg.pose.pose.position.x
            flu_y = msg.pose.pose.position.y
            flu_z = msg.pose.pose.position.z
            
            # 使用 init_vehicle_local_position 作为初始位置和 heading
            if self.init_vehicle_local_position is not None:
                # 初始位置（NED 坐标系）
                init_n = self.init_vehicle_local_position.x
                init_e = self.init_vehicle_local_position.y
                init_d = self.init_vehicle_local_position.z
                init_heading = self.init_vehicle_local_position.heading
                
                # FLU -> NED 坐标转换（当前位置）
                cur_n = flu_y
                cur_e = flu_x
                cur_d = -flu_z
                
                # 计算相对于初始位置的位置偏移（NED 坐标系）
                offset_n = cur_n - init_n
                offset_e = cur_e - init_e
                offset_d = cur_d - init_d
                
                # 根据初始 heading 进行旋转补偿
                # 旋转 -init_heading，使位置相对于初始 heading 为 0
                theta = -init_heading
                compensated_n = offset_n * math.cos(theta) + offset_e * math.sin(theta)
                compensated_e = offset_n * math.sin(theta) - offset_e * math.cos(theta)
                compensated_d = offset_d
                
                # 提取当前 yaw 角并转换四元数
                flu_qw = msg.pose.pose.orientation.w
                flu_qx = msg.pose.pose.orientation.x
                flu_qy = msg.pose.pose.orientation.y
                flu_qz = msg.pose.pose.orientation.z
                
                # FLU -> NED 四元数转换
                ned_q = self.ned_to_enu_quaternion(flu_qw, flu_qx, flu_qy, flu_qz)
                
                # 提取当前 yaw 角
                (_, _, current_yaw) = quat2euler([ned_q[0], ned_q[1], ned_q[2], ned_q[3]])
                
                # 补偿后的 yaw 角（相对于初始 heading）
                compensated_yaw = current_yaw + init_heading
                
                # 将补偿后的 yaw 角转换为四元数
                half_yaw = compensated_yaw / 2.0
                sin_half = math.sin(half_yaw)
                cos_half = math.cos(half_yaw)
                px4_msg.q = [cos_half, 0.0, 0.0, sin_half]
                
                # 发布补偿后的数据（NED 坐标系）
                px4_msg.position = [compensated_n, compensated_e, compensated_d]
                
            else:
                # 没有初始位置信息时，直接转换
                px4_msg.position = [flu_y, flu_x, -flu_z]
                px4_msg.q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.z, 
                              msg.pose.pose.orientation.x, -msg.pose.pose.orientation.y]
            
            # 速度转换：FLU -> NED
            flu_vx = msg.twist.twist.linear.x
            flu_vy = msg.twist.twist.linear.y
            flu_vz = msg.twist.twist.linear.z
            
            ned_vx = flu_vy
            ned_vy = flu_vx
            ned_vz = -flu_vz
            
            px4_msg.velocity = [ned_vx, ned_vy, ned_vz]
            
            # 角速度转换：FLU -> NED
            px4_msg.angular_velocity = [msg.twist.twist.angular.y, msg.twist.twist.angular.x, -msg.twist.twist.angular.z]
            
            # 协方差 (简化处理)
            px4_msg.position_variance = [0.0001, 0.0001, 0.0001]
            px4_msg.orientation_variance = [0.0001, 0.0001, 0.0001]
            px4_msg.velocity_variance = [0.0001, 0.0001, 0.0001]
            
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
        self.last_goal_marker_time = self.get_clock().now()
        
        # 只有在自动切换未完成或无人机已落地的情况下才重新启动切换流程
        if not self.auto_switch_enabled and not self.auto_switch_completed:
            self.get_logger().info('收到目标点标记，开始自动状态切换流程')
            self.auto_switch_enabled = True
            self.auto_switch_completed = False  # 重置完成标志
            self.auto_switch_to_egoplanner()
        elif self.auto_switch_completed and not self.auto_switch_enabled:
            # 检查无人机是否可以重新启动切换
            if self.vehicle_status is not None and self.cur_vehicle_local_position is not None:
                current_altitude = -self.cur_vehicle_local_position.z
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
                    self.get_logger().info('收到目标点标记，但无人机仍在飞行状态，忽略本次标记')
            else:
                self.get_logger().warn('收到目标点标记，但无法获取无人机状态，忽略本次标记')
    
    def get_clock_microseconds(self):
        t_ = self.get_clock().now().seconds_nanoseconds()
        return int(t_[0]*1e6 + t_[1]/1000)

    def cmd_pose_local_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
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
        cmd.yaw = yaw
        self.cmd = cmd
        
    def cmd_pose_local_flu_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_FLU"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = quat2euler(orientation_list)

        # 使用 TF 变换将 FLU -> NED
        # FLU 坐标系：机体坐标系
        # NED 坐标系：px4_odom再变换坐标系（PX4）
        # 通过 TF 变换：world -> px4_odom
        try:
            px4_odom_frame = self.namespace.lstrip('/') + 'px4_odom'
            world_frame = 'world'
            
            # 创建 PoseStamped 用于 TF 变换
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = world_frame
            pose_stamped.pose.position.x = msg.position.x
            pose_stamped.pose.position.y = msg.position.y
            pose_stamped.pose.position.z = msg.position.z
            pose_stamped.pose.orientation = msg.orientation
            
            # 使用 TF 变换将位姿从  world -> px4_odom
            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                px4_odom_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # ENU -> NED 坐标转换
            # ENU: x=East, y=North, z=Up
            # NED: x=North, y=East, z=Down
            p_n = transformed_pose.pose.position.y
            p_e = transformed_pose.pose.position.x
            p_d = -transformed_pose.pose.position.z
            
            # 航向角：使用转换后的航向角
            transformed_q = [transformed_pose.pose.orientation.w,
                           transformed_pose.pose.orientation.x,
                           transformed_pose.pose.orientation.y,
                           transformed_pose.pose.orientation.z]
            (_, _, transformed_yaw) = quat2euler(transformed_q)
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [p_n, p_e, p_d]
            cmd.yaw = 1.57 - transformed_yaw
            self.cmd = cmd
            
        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_pose_local_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时，使用当前航向角（备用方案）
            theta = self.cur_vehicle_local_position.heading if self.cur_vehicle_local_position else 0.0
            
            # Transfer position from FLU to NED, msg.position.xyz is flu, respectively
            p_n = msg.position.x * math.cos(theta) + msg.position.y * math.sin(theta)
            p_e = msg.position.x * math.sin(theta) - msg.position.y * math.cos(theta)
            p_d = -msg.position.z
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [p_n, p_e, p_d]
            cmd.yaw = self.init_vehicle_local_position.heading + yaw if self.init_vehicle_local_position else yaw
            self.cmd = cmd

    def cmd_vel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "VEL_NED"
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [math.nan, math.nan, math.nan]
        cmd.velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        cmd.yaw = math.nan
        cmd.yawspeed = msg.angular.z
        self.cmd = cmd

    def cmd_vel_flu_callback(self, msg):
        """ Let a be heading angle
        [[cos a , sin a,  0],     [f]   [n]
         [-sin a, cos a,  0],  *  [l] = [e]
         [0     , 0    , -1]]     [u]   [d]
        """
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "VEL_FLU"
        
        # 使用 TF 变换将 FLU -> NED
        # FLU 坐标系：机体坐标系
        # NED 坐标系：世界坐标系（PX4）
        # 通过 TF 变换：world -> base_footprint
        try:
            base_footprint_frame = self.namespace.lstrip('/') + 'base_footprint'
            world_frame = 'world'
            
            # 获取 world -> base_footprint 的 TF 变换
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                base_footprint_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 从 TF 变换中提取旋转矩阵
            q = transform.transform.rotation
            rotation_matrix = self.quat_to_rot(q.w, q.x, q.y, q.z)
            
            # 将速度向量从 FLU 转换到 ENU
            flu_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            enu_vel = rotation_matrix @ flu_vel
            
            # ENU -> NED 坐标转换
            # ENU: x=East, y=North, z=Up
            # NED: x=North, y=East, z=Down
            v_n = enu_vel[1]
            v_e = enu_vel[0]
            v_d = -enu_vel[2]
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [v_n, v_e, v_d]
            cmd.yaw = math.nan
            cmd.yawspeed = -msg.angular.z
            self.cmd = cmd
            
        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_vel_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时，使用当前航向角（备用方案）
            theta = self.cur_vehicle_local_position.heading if self.cur_vehicle_local_position else 0.0
            
            # Transform velocity from FLU to NED, msg.linear.xyz is flu, respectively
            v_n = msg.linear.x * math.cos(theta) + msg.linear.y * math.sin(theta)
            v_e = msg.linear.x * math.sin(theta) - msg.linear.y * math.cos(theta)
            v_d = -msg.linear.z
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [v_n, v_e, v_d]
            cmd.yaw = math.nan
            cmd.yawspeed = -msg.angular.z
            self.cmd = cmd
        
    def cmd_accel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return

        self.OFFBOARD_STATE = "ACCEL_NED"
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [math.nan, math.nan, math.nan]
        cmd.velocity = [math.nan, math.nan, math.nan]
        cmd.acceleration = [msg.linear.x, msg.linear.y, msg.linear.z]
        #TODO: How about yaw?
        self.cmd = cmd
        
    def cmd_accel_flu_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return

        self.OFFBOARD_STATE = "ACCEL_FLU"
        
        # 使用 TF 变换将 FLU -> NED
        # FLU 坐标系：机体坐标系
        # NED 坐标系：世界坐标系（PX4）
        # 通过 TF 变换：world -> base_footprint
        try:
            base_footprint_frame = self.namespace.lstrip('/') + 'base_footprint'
            world_frame = 'world'
            
            # 获取 world -> base_footprint 的 TF 变换
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                base_footprint_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 从 TF 变换中提取旋转矩阵
            q = transform.transform.rotation
            rotation_matrix = self.quat_to_rot(q.w, q.x, q.y, q.z)
            
            # 将加速度向量从 FLU 转换到 ENU
            flu_accel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            enu_accel = rotation_matrix @ flu_accel
            
            # ENU -> NED 坐标转换
            # ENU: x=East, y=North, z=Up
            # NED: x=North, y=East, z=Down
            a_n = enu_accel[1]
            a_e = enu_accel[0]
            a_d = -enu_accel[2]
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [math.nan, math.nan, math.nan]
            cmd.acceleration = [a_n, a_e, a_d]
            # How about yaw
            self.cmd = cmd
            
        except Exception as tf_error:
            self.get_logger().warning(f'TF transform failed in cmd_accel_flu: {str(tf_error)}, using current heading')
            # TF 变换失败时，使用当前航向角（备用方案）
            theta = self.cur_vehicle_local_position.heading if self.cur_vehicle_local_position else 0.0
            
            # Transform acceleration from FLU to NED, msg.linear.xyz is flu, respectively
            a_n = msg.linear.x * math.cos(theta) + msg.linear.y * math.sin(theta)
            a_e = msg.linear.x * math.sin(theta) - msg.linear.y * math.cos(theta)
            a_d = -msg.linear.z
            
            # Construct TrajectorySetpoint message
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position = [math.nan, math.nan, math.nan]
            cmd.velocity = [math.nan, math.nan, math.nan]
            cmd.acceleration = [a_n, a_e, a_d]
            # How about yaw
            self.cmd = cmd
    
    def cmd_attitude_flu_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "ATTITUDE_FLU"
        cmd = VehicleAttitudeSetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.q_d = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        cmd.thrust = msg.linear.x
        self.cmd = cmd


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
        # | Altitude (限制为1米)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_local_position.ref_lat, param6=self.cur_vehicle_local_position.ref_lon, param7=1.0)
        self.get_logger().info("Take off command send. Target altitude: 1.0m")

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

    def auto_switch_to_egoplanner(self):
        """自动切换到egoplanner控制模式"""
        if self.vehicle_status is None or self.cur_vehicle_local_position is None:
            self.get_logger().warn('无法获取无人机状态，等待数据...')
            return
        
        # 1. 检查是否已起飞（使用高度判断）
        current_altitude = -self.cur_vehicle_local_position.z  # NED坐标系，z向下为负
        is_flying = current_altitude > 0.5  # 高度超过0.5米认为已起飞
        
        # 2. 检查是否已解锁
        is_armed = self.vehicle_status.arming_state == 2  # ARMING_STATE_ARMED
        
        # 3. 检查是否在offboard模式
        is_offboard = self.vehicle_status.nav_state == 14  # NAVIGATION_STATE_OFFBOARD
        
        self.get_logger().info(f'状态检查: nav_state={self.vehicle_status.nav_state}, 高度={current_altitude:.2f}m, 已起飞={is_flying}, 已解锁={is_armed}, offboard模式={is_offboard}')
        
        # 执行状态切换（按照PX4安全要求：先offboard，再解锁，最后起飞）
        # 只有在非offboard模式时才切换
        if not is_offboard:
            self.get_logger().info('先切换到offboard模式...')
            self.OFFBOARD_STATE = "ENABLED"
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.get_logger().info('offboard模式切换完成，等待解锁')
            return
        
        # 已经在offboard模式，检查是否需要解锁
        if not is_armed:
            self.get_logger().info('无人机未解锁，执行解锁...')
            self.arm()
            self.get_logger().info('等待解锁完成...')
            return
        
        # 已解锁，检查是否需要起飞
        if not is_flying:
            self.get_logger().info('无人机未起飞，执行起飞...')
            self.takeoff()
            self.get_logger().info('等待起飞完成...')
            return
        
        # 所有条件满足，切换到egoplanner控制
        self.get_logger().info('无人机已准备就绪，可以接收egoplanner控制指令')
        self.auto_switch_enabled = False  # 重置标志
        self.auto_switch_completed = True  # 设置完成标志

    def check_auto_switch_progress(self):
        """检查自动切换进度，确保状态转换完成"""
        if self.vehicle_status is None or self.cur_vehicle_local_position is None:
            return
        
        current_altitude = -self.cur_vehicle_local_position.z
        is_armed = self.vehicle_status.arming_state == 2
        is_offboard = self.vehicle_status.nav_state == 14
        is_flying = current_altitude > 0.5  # 使用高度判断是否在飞行状态
        
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
        
        # 每5秒重新检查一次状态
        if hasattr(self, '_last_check_time'):
            elapsed = (self.get_clock().now() - self._last_check_time).nanoseconds / 1e9
            if elapsed < 5:
                return
        
        self._last_check_time = self.get_clock().now()
        self.get_logger().info(f'自动切换进度: 已解锁={is_armed}, 已起飞={is_flying}, offboard模式={is_offboard}')
        
        # 重新执行切换逻辑
        self.auto_switch_to_egoplanner()

    def check_landing_and_disarm(self):
        """检查无人机是否落地，并在确认落地后自动解除arm"""
        if self.vehicle_status is None or self.cur_vehicle_local_position is None:
            return
        
        # 获取当前高度（NED坐标系，z向下为负）
        current_altitude = -self.cur_vehicle_local_position.z
        is_armed = self.vehicle_status.arming_state == 2
        nav_state = self.vehicle_status.nav_state
        
        # 使用PX4 nav_state判断飞行状态
        # nav_state=14表示offboard模式（飞行状态）
        # nav_state=18表示降落状态
        # 使用高度和nav_state综合判断是否在飞行状态
        is_offboard = nav_state == 14
        is_landing = nav_state == 18
        is_flying = is_offboard and current_altitude > 0.5  # 在offboard模式且高度超过0.5米认为在飞行
        
        # 检测状态变化：从飞行状态变为降落状态
        if self.was_flying and is_landing and is_armed:
            # 首次检测到降落，先切换到hold模式，再切换回降落模式
            if self.landed_time is None:
                self.get_logger().info(f'检测到无人机降落，nav_state={nav_state}, 高度={current_altitude:.2f}m，切换到hold模式...')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 16)
                self.get_logger().info('切换回降落模式...')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 21)
                self.landed_time = self.get_clock().now()
                self.get_logger().info(f'等待确认降落...')
            else:
                # 检查降落确认时间（3秒）
                elapsed_time = (self.get_clock().now() - self.landed_time).nanoseconds / 1e9
                if elapsed_time >= 3.0:
                    self.get_logger().info('确认无人机已降落，自动解除arm...')
                    self.disarm()
                    self.landed_time = None
                    self.was_flying = False
        elif is_flying:
            # 无人机在飞行状态，重置降落计时器
            self.landed_time = None
            self.was_flying = True
        elif not is_flying and not is_armed:
            # 无人机已降落且已解除arm，重置状态
            self.landed_time = None
            self.was_flying = False
            # 重置自动切换完成标志，允许下次收到goal marker时重新启动
            if self.auto_switch_completed:
                self.get_logger().info('无人机已降落并解除arm，重置自动切换状态，等待新目标点')
                self.auto_switch_completed = False
        elif is_landing and not is_armed:
            # 在降落状态且已解除arm，也重置状态
            self.landed_time = None
            self.was_flying = False
            # 重置自动切换完成标志，允许下次收到goal marker时重新启动
            if self.auto_switch_completed:
                self.get_logger().info(f'无人机在降落状态(nav_state={nav_state})且已解除arm，重置自动切换状态，等待新目标点')
                self.auto_switch_completed = False
        
        # 更新飞行状态记录
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


def main():
    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description='XTDrone2 Multirotor Communication Node')

    parser.add_argument('--model', type=str, help='Vehicle type', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id, should be unique in same model', required=True)
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")
    parser.add_argument('--debug', action='store_true', help='Enable debug mode to publish vehicle state', required=False, default=False)

    args, unknown = parser.parse_known_args()

    multirotor_communication = MultirotorCommunication(args.model, args.id, args.namespace, args.debug)
    rclpy.spin(multirotor_communication)
    multirotor_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()