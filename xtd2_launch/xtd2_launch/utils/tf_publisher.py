#!/usr/bin/env python3

import platform
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import numpy as np
import threading
from collections import deque


class TfPublisher(Node):

    def __init__(self):
        # 检查主机名，设置默认namespace
        hostname = platform.node()
        if hostname == 'ywj-B250-D3A':
            default_namespace = '/x500_depth_0/'
            self.is_sim = True
        else:
            default_namespace = '/'
            self.is_sim = False
        
        super().__init__(
            'tf_publisher',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim),
                Parameter('namespace', Parameter.Type.STRING, default_namespace)
            ]
        )
        
        # 获取namespace参数
        self.declare_parameter('namespace', default_namespace)
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # 动态 TF（odom 真值）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 静态 TF（结构关系）
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # PX4 visual odometry发布器
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.px4_visual_pub = self.create_publisher(
            VehicleOdometry,
            self.namespace + 'fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        # 补偿后的轨迹发布器
        self.compensated_traj_pub = self.create_publisher(
            Pose,
            '/xtdrone2' + self.namespace + 'cmd_pose_local_ned',
            50
        )

        # 订阅 Gazebo 发布的 odometry（真值）
        self.create_subscription(
            Odometry,
            self.namespace + 'odometry',
            self.odom_callback,
            10
        )

        # 订阅 PX4 odometry
        self.create_subscription(
            VehicleOdometry,
            self.namespace + 'fmu/out/vehicle_odometry',
            self.px4_odom_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        )

        # 订阅原始轨迹（Gazebo坐标系）
        self.create_subscription(
            PoseStamped,
            '/xtdrone2/planning/raw_trajectory',
            self.raw_trajectory_callback,
            10
        )

        # 创建相机位姿发布器
        if self.is_sim:
            self.camera_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'StereoOV7251/pose',
                10
            ) 
        else:
        # 真机环境下创建mid360/pose发布器
            self.mid360_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'mid360/pose',
                10
            )

        # 补偿参数
        self.compensation_lock = threading.Lock()
        self.px4_odom_world_position = np.array([0.0, 0.0, 0.0])
        self.px4_odom_world_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.compensation_alpha = 0.8  # 低通滤波器系数
        
        # 数据缓冲区
        self.gazebo_odom_buffer = deque(maxlen=10)
        self.px4_odom_buffer = deque(maxlen=10)

        # 一次性发布所有静态 TF
        self.publish_static_transforms()

        self.get_logger().info('Enhanced TF publisher started (static + dynamic TF + PX4 visual odometry + trajectory compensation)')

    def enu_to_ned_position(self, x, y, z):
        """将ENU坐标系的位置转换为NED坐标系"""
        # ENU: x=East, y=North, z=Up
        # NED: x=North, y=East, z=Down
        return [y, x, -z]

    # def enu_to_ned_quaternion(self, qw, qx, qy, qz):
    #     """将ENU坐标系的四元数转换为NED坐标系"""
    #     # ENU到NED的旋转是90度绕X轴旋转
    #     # q_ENU_to_NED = [cos(45), sin(45), 0, 0] = [0.7071, 0.7071, 0, 0]
    #     # 但这里我们直接重新排列四元数分量
    #     # 因为ENU和NED的坐标轴关系：
    #     # ENU: (x=East, y=North, z=Up) -> NED: (x=North, y=East, z=Down)
    #     # 所以四元数需要重新映射
    #     return [qw, qz, qx, -qy]


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

    def enu_to_ned_quaternion(self, qw, qx, qy, qz):
        R_enu = self.quat_to_rot(qw, qx, qy, qz)

        T = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        R_ned = T @ R_enu @ T.T

        T_yaw = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])

        r_ned_final = T_yaw @ R_ned

        return self.rot_to_quat(r_ned_final)

    def enu_to_ned_velocity(self, vx, vy, vz):
        """将ENU坐标系的速度转换为NED坐标系"""
        return [vy, vx, -vz]

    def enu_to_ned_angular_velocity(self, wx, wy, wz):
        """将ENU坐标系的角速度转换为NED坐标系"""
        return [wy, wx, -wz]

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

        T_yaw = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])

        r_enu_final = T_yaw.T @ (T @ R_ned @ T.T)

        return self.rot_to_quat(r_enu_final)

    def calculate_compensation(self):
        """计算px4_odom在world坐标系下的位置，作为补偿目标"""
        with self.compensation_lock:
            if len(self.gazebo_odom_buffer) == 0 or len(self.px4_odom_buffer) == 0:
                return
            
            # 获取最新的数据
            gazebo_odom = self.gazebo_odom_buffer[-1]
            px4_odom = self.px4_odom_buffer[-1]
            
            # 提取Gazebo位置和姿态（ENU坐标系）
            gazebo_pos = np.array([
                gazebo_odom['position'][0],
                gazebo_odom['position'][1], 
                gazebo_odom['position'][2]
            ])
            
            gazebo_quat = gazebo_odom['orientation']
            
            # 提取PX4位置和姿态（ENU坐标系）
            px4_pos_enu = np.array([
                px4_odom['position'][0],
                px4_odom['position'][1],
                px4_odom['position'][2]
            ])
            
            px4_quat = px4_odom['orientation']
            
            # 检查NaN值
            if any(np.isnan(gazebo_pos)) or any(np.isnan(px4_pos_enu)):
                return
            if any(np.isnan(gazebo_quat)) or any(np.isnan(px4_quat)):
                return
            
            # 计算Gazebo和PX4之间的相对变换
            # px4_odom在world坐标系下的位置 = gazebo_pos - (px4_pos_enu - gazebo_pos)
            # 简化：假设px4_odom和gazebo_odom应该重合，所以补偿目标就是gazebo位置
            target_position = gazebo_pos
            target_orientation = gazebo_quat
            
            # 应用低通滤波器
            self.px4_odom_world_position = (
                self.compensation_alpha * self.px4_odom_world_position + 
                (1.0 - self.compensation_alpha) * target_position
            )
            
            self.px4_odom_world_orientation = [
                self.compensation_alpha * self.px4_odom_world_orientation[i] + 
                (1.0 - self.compensation_alpha) * target_orientation[i]
                for i in range(4)
            ]
            
            # self.get_logger().info(
            #     f'Compensation calculated: position=({self.position_compensation[0]:.3f}, '
            #     f'{self.position_compensation[1]:.3f}, {self.position_compensation[2]:.3f})',
            #     throttle_duration_sec=2.0
            # )

    def publish_compensation_tf(self):
        """发布 world -> px4_odom tf (补偿目标)"""
        with self.compensation_lock:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = self.namespace.lstrip('/') + 'px4_odom'
            
            t.transform.translation.x = float(self.px4_odom_world_position[0])
            t.transform.translation.y = float(self.px4_odom_world_position[1])
            t.transform.translation.z = float(self.px4_odom_world_position[2])
            
            t.transform.rotation.w = float(self.px4_odom_world_orientation[0])
            t.transform.rotation.x = float(self.px4_odom_world_orientation[1])
            t.transform.rotation.y = float(self.px4_odom_world_orientation[2])
            t.transform.rotation.z = float(self.px4_odom_world_orientation[3])
            
            self.tf_broadcaster.sendTransform(t)

    def raw_trajectory_callback(self, msg: PoseStamped):
        """处理原始轨迹（直接转换坐标，不进行补偿，补偿通过tf变换链实现）"""
        try:
            # 获取原始轨迹位置（Gazebo坐标系/ENU）
            raw_position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            
            # 获取原始轨迹姿态
            raw_quat = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            
            # 将ENU位置转换为NED坐标系
            ned_position = self.enu_to_ned_position(
                raw_position[0],
                raw_position[1],
                raw_position[2]
            )
            
            # 转换姿态到NED坐标系
            ned_orientation = self.enu_to_ned_quaternion(
                raw_quat[0],
                raw_quat[1],
                raw_quat[2],
                raw_quat[3]
            )
            
            # 创建轨迹消息
            trajectory_pose = Pose()
            trajectory_pose.position.x = ned_position[0]
            trajectory_pose.position.y = ned_position[1]
            trajectory_pose.position.z = ned_position[2]
            
            trajectory_pose.orientation.w = ned_orientation[0]
            trajectory_pose.orientation.x = ned_orientation[1]
            trajectory_pose.orientation.y = ned_orientation[2]
            trajectory_pose.orientation.z = ned_orientation[3]
            
            # 发布轨迹
            self.compensated_traj_pub.publish(trajectory_pose)
            
            self.get_logger().debug(
                f'Trajectory: position=({ned_position[0]:.3f}, '
                f'{ned_position[1]:.3f}, {ned_position[2]:.3f})',
                throttle_duration_sec=1.0
            )
                
        except Exception as e:
            self.get_logger().error(f'Error processing trajectory: {str(e)}')

    def odom_callback(self, msg: Odometry):
        """Gazebo odometry callback"""
        # 不再发布 odom -> base_footprint TF，避免与 px4_odom 冲突
        # 补偿通过 tf 变换链 world -> px4_odom -> base_footprint 实现

        # 存储Gazebo odometry数据（用于计算补偿）
        gazebo_data = {
            'timestamp': msg.header.stamp,
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
                          msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        }
        
        with self.compensation_lock:
            self.gazebo_odom_buffer.append(gazebo_data)
        
        # 计算补偿
        self.calculate_compensation()
        
        # 模拟环境下发布相机位姿
        if self.is_sim:
            self.publish_camera_pose(msg)
        else: 
        # 真机环境下发布mid360/pose
            self.publish_mid360_pose(msg)
        
        # 转发到PX4 visual odometry
        self.publish_px4_visual_odometry(msg)

    def px4_odom_callback(self, msg: VehicleOdometry):
        """PX4 odometry callback"""
        # 转换坐标系
        enu_position = self.ned_to_enu_position(msg.position[0], msg.position[1], msg.position[2])
        enu_orientation = self.ned_to_enu_quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        
        # 检查NaN值
        if any(np.isnan(enu_position)) or any(np.isnan(enu_orientation)):
            self.get_logger().warning('NaN detected in PX4 odometry conversion, skipping this message')
            return
        
        # 确保值是有效的float类型
        enu_position = [float(x) for x in enu_position]
        enu_orientation = [float(x) for x in enu_orientation]
        
        # 存储转换后的ENU坐标数据
        px4_data = {
            'timestamp': msg.timestamp,
            'position': enu_position,
            'orientation': enu_orientation
        }
        
        with self.compensation_lock:
            self.px4_odom_buffer.append(px4_data)
        
        # 发布 px4_odom -> base_link tf (相对变换)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.namespace.lstrip('/') + 'px4_odom'
        t.child_frame_id = self.namespace.lstrip('/') + 'base_footprint'
        
        t.transform.translation.x = enu_position[0]
        t.transform.translation.y = enu_position[1]
        t.transform.translation.z = enu_position[2]
        
        t.transform.rotation.w = enu_orientation[0]
        t.transform.rotation.x = enu_orientation[1]
        t.transform.rotation.y = enu_orientation[2]
        t.transform.rotation.z = enu_orientation[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # 计算补偿
        self.calculate_compensation()
        
        # 发布 world -> px4_odom tf (补偿目标)
        self.publish_compensation_tf()

    def publish_px4_visual_odometry(self, msg: Odometry):
        """转换并发布PX4 visual odometry"""
        try:
            px4_msg = VehicleOdometry()
            
            # 时间戳 - 转换为PX4时间基准（微秒）
            px4_msg.timestamp = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)
            px4_msg.timestamp_sample = px4_msg.timestamp
            
            # 坐标系设置
            px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            
            # 提取原始数据
            pos_x = msg.pose.pose.position.x
            pos_y = msg.pose.pose.position.y
            pos_z = msg.pose.pose.position.z
            
            ori_w = msg.pose.pose.orientation.w
            ori_x = msg.pose.pose.orientation.x
            ori_y = msg.pose.pose.orientation.y
            ori_z = msg.pose.pose.orientation.z
            
            vel_x = msg.twist.twist.linear.x
            vel_y = msg.twist.twist.linear.y
            vel_z = msg.twist.twist.linear.z
            
            ang_vel_x = msg.twist.twist.angular.x
            ang_vel_y = msg.twist.twist.angular.y
            ang_vel_z = msg.twist.twist.angular.z
            
            # 位置转换: ENU -> NED
            px4_msg.position = self.enu_to_ned_position(pos_x, pos_y, pos_z)
            
            # 四元数转换: ENU -> NED
            px4_msg.q = self.enu_to_ned_quaternion(ori_w, ori_x, ori_y, ori_z)
            
            # 速度转换: ENU -> NED
            px4_msg.velocity = self.enu_to_ned_velocity(vel_x, vel_y, vel_z)
            
            # 角速度转换: ENU -> NED
            px4_msg.angular_velocity = self.enu_to_ned_angular_velocity(ang_vel_x, ang_vel_y, ang_vel_z)
            
            # 协方差 (简化处理)
            px4_msg.position_variance = [0.0001, 0.0001, 0.0001]
            px4_msg.orientation_variance = [0.0001, 0.0001, 0.0001]
            px4_msg.velocity_variance = [0.0001, 0.0001, 0.0001]
            
            # 质量指标
            px4_msg.quality = 100  # 最高质量
            px4_msg.reset_counter = 0
            
            # 发布消息
            self.px4_visual_pub.publish(px4_msg)
            
            self.get_logger().debug(
                f'PX4 Visual Odom: pos=[{px4_msg.position[0]:.2f}, {px4_msg.position[1]:.2f}, {px4_msg.position[2]:.2f}]',
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting to PX4 visual odometry: {str(e)}')

    def publish_camera_pose(self, odom_msg: Odometry):
        """从odom推算相机位姿并发布"""
        camera_pose = PoseStamped()
        camera_pose.header.stamp = odom_msg.header.stamp
        camera_pose.header.frame_id = self.namespace.lstrip('/') + 'odom'
        
        # 相机相对于base_footprint的固定偏移
        camera_offset_x = 0.12  # 前方偏移
        camera_offset_y = 0.03  # 侧向偏移
        camera_offset_z = 0.242  # 下方偏移
        
        # 计算相机在世界坐标系中的位置
        import math
        
        # 获取无人机的朝向（四元数）
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # 简化的旋转计算
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        # 旋转相机偏移
        rotated_offset_x = camera_offset_x * math.cos(yaw) - camera_offset_y * math.sin(yaw)
        rotated_offset_y = camera_offset_x * math.sin(yaw) + camera_offset_y * math.cos(yaw)
        rotated_offset_z = camera_offset_z
        
        # 计算相机在世界坐标系中的位置
        camera_pose.pose.position.x = odom_msg.pose.pose.position.x + rotated_offset_x
        camera_pose.pose.position.y = odom_msg.pose.pose.position.y + rotated_offset_y
        camera_pose.pose.position.z = odom_msg.pose.pose.position.z + rotated_offset_z
        
        # 相机朝向与无人机朝向一致
        camera_pose.pose.orientation = odom_msg.pose.pose.orientation
        
        # 发布相机位姿
        self.camera_pose_publisher.publish(camera_pose)
        
        # 发布相机TF
        camera_tf = TransformStamped()
        camera_tf.header.stamp = odom_msg.header.stamp
        camera_tf.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
        camera_tf.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
        
        camera_tf.transform.translation.x = camera_offset_x
        camera_tf.transform.translation.y = camera_offset_y
        camera_tf.transform.translation.z = camera_offset_z
        
        # 相机相对于 base_footprint 
        camera_tf.transform.rotation.w = 1.0
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(camera_tf)

    def publish_mid360_pose(self, odom_msg: Odometry):
        """发布mid360/pose话题"""
        mid360_pose = PoseStamped()
        mid360_pose.header.stamp = odom_msg.header.stamp
        mid360_pose.header.frame_id = 'livox_frame'
        
        # mid360相对于base_link的固定偏移
        mid360_offset_x = -0.1  # 后方偏移
        mid360_offset_y = 0.0  # 侧向偏移
        mid360_offset_z = -0.1  # 下方偏移
        
        # 计算mid360在世界坐标系中的位置
        import math
        
        # 获取无人机的朝向（四元数）
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # 简化的旋转计算
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        # 旋转mid360偏移
        rotated_offset_x = mid360_offset_x * math.cos(yaw) - mid360_offset_y * math.sin(yaw)
        rotated_offset_y = mid360_offset_x * math.sin(yaw) + mid360_offset_y * math.cos(yaw)
        rotated_offset_z = mid360_offset_z
        
        # 计算mid360在世界坐标系中的位置
        mid360_pose.pose.position.x = odom_msg.pose.pose.position.x + rotated_offset_x
        mid360_pose.pose.position.y = odom_msg.pose.pose.position.y + rotated_offset_y
        mid360_pose.pose.position.z = odom_msg.pose.pose.position.z + rotated_offset_z
        
        # mid360朝向与无人机朝向一致
        mid360_pose.pose.orientation = odom_msg.pose.pose.orientation
        
        # 发布mid360位姿
        self.mid360_pose_publisher.publish(mid360_pose)

    def publish_static_transforms(self):
        """发布静态TF变换"""
        now = self.get_clock().now().to_msg() 
        while now.sec == 0:  # 等待 /clock 开始发布
            rclpy.spin_once(self, timeout_sec=1.0)
            now = self.get_clock().now().to_msg()
            self.get_logger().info(f'now:{now}')

        tfs = []

        # world -> map
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # map -> odom
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        if self.namespace != '/':
            # odom -> namespace/odom
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = self.namespace.lstrip('/') + 'odom'
            t.transform.rotation.w = 1.0
            tfs.append(t)

        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
        t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        

        if self.is_sim:
            # base_link -> OakD-Lite base
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.transform.translation.x = 0.12
            t.transform.translation.y = 0.03
            t.transform.translation.z = 0.242
            t.transform.rotation.w = 1.0
            tfs.append(t)
    
            # OakD-Lite -> StereoOV7251（点云）
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
            t.transform.translation.x = 0.01233
            t.transform.translation.y = -0.03
            t.transform.translation.z = 0.01878
            t.transform.rotation.x = 0.707
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.707
            t.transform.rotation.w = 0.0
            tfs.append(t)
    
            # OakD-Lite -> IMX214（RGB）
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'IMX214'
            t.transform.translation.x = 0.01233
            t.transform.translation.y = -0.03
            t.transform.translation.z = 0.01878
            t.transform.rotation.w = 1.0
            tfs.append(t)
    
            # StereoOV7251 -> StereoOV7251
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
            t.child_frame_id = self.namespace.lstrip('/') + 'StereoOV7251'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        # 真机环境下添加mid360到base_link的TF变换
        else:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'livox_frame'
            t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
            t.transform.translation.x = -0.1
            t.transform.translation.y = 0.0
            t.transform.translation.z = -0.1
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = -0.87266
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        self.static_tf_broadcaster.sendTransform(tfs)

        self.get_logger().info('Static TFs published')


def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()