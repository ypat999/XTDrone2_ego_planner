#!/usr/bin/env python3

import platform
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
import math


class TfPublisher(Node):

    def __init__(self):
        # 检查主机名，设置默认namespace
        hostname = platform.node()
        if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
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

        # # 补偿后的轨迹发布器
        # self.compensated_traj_pub = self.create_publisher(
        #     Pose,
        #     '/xtdrone2' + self.namespace + 'cmd_pose_local_ned',
        #     50
        # )

        # 订阅 Gazebo 发布的 odometry（真值）
        self.create_subscription(
            Odometry,
            self.namespace + 'odometry',
            self.odom_callback,
            10
        )

        # # 订阅原始轨迹（Gazebo坐标系）
        # self.create_subscription(
        #     PoseStamped,
        #     '/xtdrone2/planning/raw_trajectory',
        #     self.raw_trajectory_callback,
        #     10
        # )

        # 创建相机位姿发布器
        if self.is_sim:
            self.camera_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'StereoOV7251/pose',
                10
            )
            self.mid360_down_odom_publisher = self.create_publisher(
                Odometry,
                self.namespace + 'livox_down_frame/mid360_down_lidar/odometry',
                10
            ) 
        else:
        # 真机环境下创建mid360/pose发布器
            self.mid360_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'mid360/pose',
                10
            )

        # 一次性发布所有静态 TF
        self.publish_static_transforms()

        self.get_logger().info('TF publisher started (static + dynamic TF + PX4 visual odometry)')


    def odom_callback(self, msg: Odometry):
        """Gazebo odometry callback - 发布 world -> base_footprint tf (真值)"""
        # 发布 world -> base_footprint tf (使用Gazebo odometry作为真值)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = self.namespace.lstrip('/') + 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        
        # 模拟环境下发布相机位姿
        if self.is_sim:
            self.publish_camera_pose(msg)
            self.publish_mid360_down_odom(msg)
        else: 
        # 真机环境下发布mid360/pose
            self.publish_mid360_pose(msg)

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

    def publish_mid360_down_odom(self, odom_msg: Odometry):
        """从 base_link odometry 推算 mid360_down 雷达 odometry 并发布

        将 /x500_depth_0/odometry (world -> base_footprint) 通过静态 TF
        (base_link -> livox_down_frame/mid360_down_lidar) 变换为
        odom -> mid360_down_lidar 的 odometry。
        """
        
        # print("publish_mid360_down_odom")

        mid360_down_odom = Odometry()
        mid360_down_odom.header.stamp = odom_msg.header.stamp
        mid360_down_odom.header.frame_id = '/x500_depth_0/odom'
        mid360_down_odom.child_frame_id = '/x500_depth_0/livox_down_frame/mid360_down_lidar'

        # 静态 TF: base_link -> livox_down_frame/mid360_down_lidar
        # translation: (0.1, 0, 0.2)  rotation: euler(0°, 150°, 0°)
        offset_x, offset_y, offset_z = 0.1, 0.0, 0.2
        qx_s, qy_s, qz_s, qw_s = self.euler_to_quaternion(0, 150, 0)

        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w

        # 将静态偏移旋转到世界坐标系 (base_footprint -> base_link 为 identity，直接用 yaw 旋转)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        rx = offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
        ry = offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
        rz = offset_z

        # 位置: world_pos_mid360 = world_pos_base + R_world_base * offset
        mid360_down_odom.pose.pose.position.x = odom_msg.pose.pose.position.x + rx
        mid360_down_odom.pose.pose.position.y = odom_msg.pose.pose.position.y + ry
        mid360_down_odom.pose.pose.position.z = odom_msg.pose.pose.position.z + rz

        # 朝向: q_result = q_odom * q_static  (world -> base * base -> mid360 = world -> mid360)
        w = qw * qw_s - qx * qx_s - qy * qy_s - qz * qz_s
        x = qw * qx_s + qx * qw_s + qy * qz_s - qz * qy_s
        y = qw * qy_s - qx * qz_s + qy * qw_s + qz * qx_s
        z = qw * qz_s + qx * qy_s - qy * qx_s + qz * qw_s
        mid360_down_odom.pose.pose.orientation.x = x
        mid360_down_odom.pose.pose.orientation.y = y
        mid360_down_odom.pose.pose.orientation.z = z
        mid360_down_odom.pose.pose.orientation.w = w

        # 速度变换: 将 base_footprint 系的速度旋转到 mid360_down 系
        # R_s 为 pitch(150°) 的旋转矩阵，R_s^T = pitch(-150°)
        pitch_s = math.radians(150)
        cos_p = math.cos(pitch_s)
        sin_p = math.sin(pitch_s)

        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        vz = odom_msg.twist.twist.linear.z
        mid360_down_odom.twist.twist.linear.x = cos_p * vx - sin_p * vz
        mid360_down_odom.twist.twist.linear.y = vy
        mid360_down_odom.twist.twist.linear.z = sin_p * vx + cos_p * vz

        wx = odom_msg.twist.twist.angular.x
        wy = odom_msg.twist.twist.angular.y
        wz = odom_msg.twist.twist.angular.z
        mid360_down_odom.twist.twist.angular.x = cos_p * wx - sin_p * wz
        mid360_down_odom.twist.twist.angular.y = wy
        mid360_down_odom.twist.twist.angular.z = sin_p * wx + cos_p * wz

        self.mid360_down_odom_publisher.publish(mid360_down_odom)
        # print("published_mid360_down_odom")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角（角度）转换为四元数
        
        Args:
            roll: 绕X轴旋转角度（度）
            pitch: 绕Y轴旋转角度（度）
            yaw: 绕Z轴旋转角度（度）
            
        Returns:
            四元数 (x, y, z, w)
        """
        import math
        
        # 将角度转换为弧度
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qx, qy, qz, qw)

    def publish_static_transforms(self):
        """发布静态TF变换"""
        now = self.get_clock().now().to_msg() 
        while now.sec == 0:  # 等待 /clock 开始发布
            rclpy.spin_once(self, timeout_sec=1.0)
            now = self.get_clock().now().to_msg()
            self.get_logger().info(f'now:{now}')

        tfs = []


        if self.namespace != '/':
            # odom -> namespace/odom
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = self.namespace.lstrip('/') + 'odom'
            t.transform.rotation.w = 1.0
            tfs.append(t)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'odom'
            t.child_frame_id = 'world'
            t.transform.rotation.w = 1.0
            tfs.append(t)
        else:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'world'
            t.transform.rotation.w = 1.0
            tfs.append(t)

        if self.is_sim:
            # map -> odom
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.rotation.w = 1.0
            tfs.append(t)


            # base_footprint -> base_link
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
            t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
            t.transform.rotation.w = 1.0
            tfs.append(t)

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

            # x500_depth_0/livox_frame/mid360_lidar
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'livox_frame/mid360_lidar'
            t.transform.translation.x = 0.1  # 0.1 0 0.30 0 -0.5236 3.1415926
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.3
            qx, qy, qz, qw = self.euler_to_quaternion(0, 30, 0)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            tfs.append(t)

            # x500_depth_0/livox_down_frame/mid360_down_lidar
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'livox_down_frame/mid360_down_lidar'
            t.transform.translation.x = 0.1  # 0.1 0 0.30 0 -0.5236 3.1415926
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2
            qx, qy, qz, qw = self.euler_to_quaternion(0, 150, 0)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            tfs.append(t)

        # # 真机环境下添加mid360到base_link的TF变换
        # else:
        #     t = TransformStamped()
        #     t.header.stamp = now
        #     t.header.frame_id = 'livox_frame'
        #     t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
        #     t.transform.translation.x = -0.1
        #     t.transform.translation.y = 0.0
        #     t.transform.translation.z = -0.1
        #     t.transform.rotation.x = 0.0
        #     t.transform.rotation.y = -0.87266
        #     t.transform.rotation.z = 0.0
        #     t.transform.rotation.w = 1.0
        #     tfs.append(t)

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