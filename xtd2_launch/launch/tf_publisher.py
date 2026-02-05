#!/usr/bin/env python3

from curses import noraw
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from px4_msgs.msg import VehicleOdometry
import numpy as np


class TfPublisher(Node):

    def __init__(self):
        super().__init__(
            'tf_publisher',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )

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
            '/x500_depth_0/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        # 订阅 Gazebo 发布的 odometry（真值）
        self.create_subscription(
            Odometry,
            '/x500_depth_0/odometry',
            self.odom_callback,
            10
        )

        # 创建相机位姿发布器
        self.camera_pose_publisher = self.create_publisher(
            PoseStamped,
            '/x500_depth_0/StereoOV7251/pose',
            10
        )

        # 一次性发布所有静态 TF
        self.publish_static_transforms()

        self.get_logger().info('TF publisher started (static + dynamic TF + PX4 visual odometry)')


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
        w = np.sqrt(1 + np.trace(R)) / 2
        x = (R[2,1] - R[1,2]) / (4*w)
        y = (R[0,2] - R[2,0]) / (4*w)
        z = (R[1,0] - R[0,1]) / (4*w)
        return [w, x, y, z]

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

    # =========================
    # 动态 TF（Gazebo 真值）
    # =========================
    def odom_callback(self, msg: Odometry):
        """
        Gazebo 真值：
        x500_depth_0/odom -> x500_depth_0/base_footprint
        同时推算相机位姿并发布
        转发到PX4 visual odometry
        """
        # 发布 odom -> base_footprint TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'x500_depth_0/odom'
        t.child_frame_id = 'x500_depth_0/base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        # 发布相机位姿
        self.publish_camera_pose(msg)
        
        # 转发到PX4 visual odometry
        self.publish_px4_visual_odometry(msg)


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
            
            # 日志记录（限制频率）
            self.get_logger().debug(
                f'PX4 Visual Odom: pos=[{px4_msg.position[0]:.2f}, {px4_msg.position[1]:.2f}, {px4_msg.position[2]:.2f}], '
                f'vel=[{px4_msg.velocity[0]:.2f}, {px4_msg.velocity[1]:.2f}, {px4_msg.velocity[2]:.2f}]',
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting to PX4 visual odometry: {str(e)}')


    def publish_camera_pose(self, odom_msg: Odometry):
        """
        从odom推算相机位姿并发布
        相机相对于base_footprint的固定变换：
        - 位置：前方0.1米，下方0.05米（相对于无人机中心）
        - 朝向：与无人机朝向一致
        """
        # 创建相机位姿消息
        camera_pose = PoseStamped()
        camera_pose.header.stamp = odom_msg.header.stamp
        camera_pose.header.frame_id = 'x500_depth_0/odom'
        
        # 相机相对于base_footprint的固定偏移
        # 假设相机位于无人机.12 .03 .242
        camera_offset_x = 0.12  # 前方偏移
        camera_offset_y = 0.03  # 侧向偏移
        camera_offset_z = 0.242  # 下方偏移
        
        # 计算相机在世界坐标系中的位置
        # 使用odom的位姿加上相机相对于无人机的偏移
        import math
        
        # 获取无人机的朝向（四元数）
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # 将相机偏移旋转到世界坐标系
        # 计算旋转矩阵
        # 简化的旋转计算（假设无人机主要在水平面运动）
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
        # 相机相对于无人机绕 Y 轴旋转 90°，将该相对旋转应用到 odom 的朝向上
        qw = odom_msg.pose.pose.orientation.w
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z

        # 相对旋转四元数（绕 y 轴 90°）
        import math
        # rot_w = math.cos(-math.pi / 4)
        rot_w = 1.0
        rot_x = 0.0
        # rot_y = math.sin(-math.pi / 4)
        rot_y = 0.0
        rot_z = 0.0

        def quat_mult(a, b):
            aw, ax, ay, az = a
            bw, bx, by, bz = b
            return [
                aw*bw - ax*bx - ay*by - az*bz,
                aw*bx + ax*bw + ay*bz - az*by,
                aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw
            ]

        cam_q = quat_mult([qw, qx, qy, qz], [rot_w, rot_x, rot_y, rot_z])
        camera_pose.pose.orientation.w = cam_q[0]
        camera_pose.pose.orientation.x = cam_q[1]
        camera_pose.pose.orientation.y = cam_q[2]
        camera_pose.pose.orientation.z = cam_q[3]
        
        # 发布相机位姿
        self.camera_pose_publisher.publish(camera_pose)
        
        # 可选：发布相机TF
        camera_tf = TransformStamped()
        camera_tf.header.stamp = odom_msg.header.stamp
        camera_tf.header.frame_id = 'x500_depth_0/base_footprint'
        camera_tf.child_frame_id = 'x500_depth_0/OakD-Lite/base_link/StereoOV7251' #'x500_depth_0/StereoOV7251'
        
        camera_tf.transform.translation.x = camera_offset_x
        camera_tf.transform.translation.y = camera_offset_y
        camera_tf.transform.translation.z = camera_offset_z
        
        # 相机相对于无人机的旋转（这里假设相机与无人机朝向一致）
        # 相机相对于 base_footprint 绕 Y 轴旋转 90°
        camera_tf.transform.rotation.w = rot_w
        camera_tf.transform.rotation.x = rot_x
        camera_tf.transform.rotation.y = rot_y
        camera_tf.transform.rotation.z = rot_z
        
        self.tf_broadcaster.sendTransform(camera_tf)


    # def pointcloud_callback(self, msg):
    #     """
    #     点云重发布回调函数
    #     将点云数据的坐标系从'x500_depth_0/StereoOV7251'修改为'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
    #     """
    #     # 创建新的点云消息
    #     republished_msg = PointCloud2()
        
    #     # 复制所有字段
    #     republished_msg.header = msg.header
    #     republished_msg.height = msg.height
    #     republished_msg.width = msg.width
    #     republished_msg.fields = msg.fields
    #     republished_msg.is_bigendian = msg.is_bigendian
    #     republished_msg.point_step = msg.point_step
    #     republished_msg.row_step = msg.row_step
    #     republished_msg.data = msg.data
    #     republished_msg.is_dense = msg.is_dense
        
    #     # 修改坐标系
    #     republished_msg.header.frame_id = 'x500_depth_0/StereoOV7251'
        
    #     # 发布重发布的点云
    #     self.pointcloud_publisher.publish(republished_msg)
        
    #     # 可选：记录日志
    #     self.get_logger().debug('Republished pointcloud with corrected frame_id', throttle_duration_sec=5.0)

    # =========================
    # 静态 TF       （一次性）
    # =========================
    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg() 
        while now.sec == 0:  # 等待 /clock 开始发布
            rclpy.spin_once(self, timeout_sec=0.01)
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

        # odom -> x500_depth_0/odom
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'x500_depth_0/odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'x500_depth_0/base_footprint'
        t.child_frame_id = 'x500_depth_0/base_link'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # base_link -> OakD-Lite base
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'x500_depth_0/base_link'
        t.child_frame_id = 'x500_depth_0/OakD-Lite/base_link'
        t.transform.translation.x = 0.12
        t.transform.translation.y = 0.03
        t.transform.translation.z = 0.242
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # OakD-Lite -> StereoOV7251（点云）
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'x500_depth_0/OakD-Lite/base_link'
        t.child_frame_id = 'x500_depth_0/OakD-Lite/base_link/StereoOV7251' #'x500_depth_0/StereoOV7251'
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
        t.header.frame_id = 'x500_depth_0/OakD-Lite/base_link'
        t.child_frame_id = 'x500_depth_0/IMX214'
        t.transform.translation.x = 0.01233
        t.transform.translation.y = -0.03
        t.transform.translation.z = 0.01878
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # StereoOV7251 -> StereoOV7251
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
        t.child_frame_id = 'x500_depth_0/StereoOV7251'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
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