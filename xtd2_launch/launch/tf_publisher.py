#!/usr/bin/env python3

from curses import noraw
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


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

        # # 点云重发布功能
        # # 订阅原始点云话题
        # self.pointcloud_subscription = self.create_subscription(
        #     PointCloud2,
        #     '/x500_depth_0/StereoOV7251/pointcloud',
        #     self.pointcloud_callback,
        #     10)
        
        # # 创建重发布的点云话题
        # self.pointcloud_publisher = self.create_publisher(
        #     PointCloud2,
        #     '/x500_depth_0/StereoOV7251/pointcloud_republished',
        #     10)

        # 一次性发布所有静态 TF
        self.publish_static_transforms()

        # 定时发布静态TF
        # self.timer = self.create_timer(0.1, self.publish_static_transforms)

        self.get_logger().info('TF publisher started (static + dynamic TF)')


    # =========================
    # 动态 TF（Gazebo 真值）
    # =========================
    def odom_callback(self, msg: Odometry):
        """
        Gazebo 真值：
        x500_depth_0/odom -> x500_depth_0/base_footprint
        同时推算相机位姿并发布
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
        camera_pose.pose.orientation = odom_msg.pose.pose.orientation
        
        # 发布相机位姿
        self.camera_pose_publisher.publish(camera_pose)
        
        # 可选：发布相机TF
        camera_tf = TransformStamped()
        camera_tf.header.stamp = odom_msg.header.stamp
        camera_tf.header.frame_id = 'x500_depth_0/base_footprint'
        camera_tf.child_frame_id = 'x500_depth_0/StereoOV7251'
        
        camera_tf.transform.translation.x = camera_offset_x
        camera_tf.transform.translation.y = camera_offset_y
        camera_tf.transform.translation.z = camera_offset_z
        
        # 相机相对于无人机的旋转（这里假设相机与无人机朝向一致）
        camera_tf.transform.rotation.w = 1.0  # 无旋转
        
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

        # # x500_depth_0/odom -> x500_depth_0/base_footprint
        # t = TransformStamped()
        # t.header.stamp = now
        # t.header.frame_id = 'x500_depth_0/odom'
        # t.child_frame_id = 'x500_depth_0/base_footprint'
        # t.transform.rotation.w = 1.0
        # tfs.append(t)

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
        t.child_frame_id = 'x500_depth_0/StereoOV7251'
        t.transform.translation.x = 0.01233
        t.transform.translation.y = -0.03
        t.transform.translation.z = 0.01878
        t.transform.rotation.w = 1.0
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
        t.header.frame_id = 'x500_depth_0/StereoOV7251'
        t.child_frame_id = 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        tfs.append(t)

        self.static_tf_broadcaster.sendTransform(tfs)
        # self.tf_broadcaster.sendTransform(tfs)

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
