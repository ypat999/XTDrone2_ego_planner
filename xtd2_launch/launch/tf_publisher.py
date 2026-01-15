#!/usr/bin/env python3

from curses import noraw
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
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

        # 点云重发布功能
        # 订阅原始点云话题
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/x500_depth_0/StereoOV7251/pointcloud',
            self.pointcloud_callback,
            10)
        
        # 创建重发布的点云话题
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/x500_depth_0/StereoOV7251/pointcloud_republished',
            10)

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
        """
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'x500_depth_0/odom'
        t.child_frame_id = 'x500_depth_0/base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


    def pointcloud_callback(self, msg):
        """
        点云重发布回调函数
        将点云数据的坐标系从'x500_depth_0/StereoOV7251'修改为'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
        """
        # 创建新的点云消息
        republished_msg = PointCloud2()
        
        # 复制所有字段
        republished_msg.header = msg.header
        republished_msg.height = msg.height
        republished_msg.width = msg.width
        republished_msg.fields = msg.fields
        republished_msg.is_bigendian = msg.is_bigendian
        republished_msg.point_step = msg.point_step
        republished_msg.row_step = msg.row_step
        republished_msg.data = msg.data
        republished_msg.is_dense = msg.is_dense
        
        # 修改坐标系
        republished_msg.header.frame_id = 'x500_depth_0/StereoOV7251'
        
        # 发布重发布的点云
        self.pointcloud_publisher.publish(republished_msg)
        
        # 可选：记录日志
        self.get_logger().debug('Republished pointcloud with corrected frame_id', throttle_duration_sec=5.0)

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
