#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅odometry话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/x500_depth_0/odometry',
            self.odom_callback,
            10)
        
        # 定时发布静态TF
        self.timer = self.create_timer(0.1, self.publish_static_tf)
        
        self.get_logger().info('TF Publisher started')
    
    def odom_callback(self, msg):
        # 发布x500_depth_0/odom到x500_depth_0/base_footprint的TF（来自odometry消息）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'x500_depth_0/odom'
        t.child_frame_id = 'x500_depth_0/base_footprint'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        
        # 发布x500_depth_0/base_footprint到x500_depth_0/base_link的静态TF
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'x500_depth_0/base_footprint'
        t2.child_frame_id = 'x500_depth_0/base_link'
        
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)
        
        # 发布odom到x500_depth_0/odom的连接TF（将全局odom连接到特定命名空间的odom）
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'odom'
        t3.child_frame_id = 'x500_depth_0/odom'
        
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0
        
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t3)
    
    def publish_static_tf(self):
        # 发布map到odom的静态TF（初始位置）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
        # 发布x500_depth_0/base_link到OakD-Lite基座的TF
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'x500_depth_0/base_link'
        t1.child_frame_id = 'x500_depth_0/OakD-Lite/base_link'
        
        t1.transform.translation.x = 0.12  # OakD-Lite在飞机前方12cm
        t1.transform.translation.y = 0.03  # 右侧3cm
        t1.transform.translation.z = 0.242  # 上方24.2cm
        
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t1)
        
        # 发布x500_depth_0/OakD-Lite/base_link到IMX214相机的TF
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'x500_depth_0/OakD-Lite/base_link'
        t2.child_frame_id = 'x500_depth_0/OakD-Lite/base_link/IMX214'
        
        t2.transform.translation.x = 0.01233  # 相机在基座上的位置
        t2.transform.translation.y = -0.03
        t2.transform.translation.z = 0.01878
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)
        
        # 发布x500_depth_0/OakD-Lite/base_link到StereoOV7251相机的TF
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'x500_depth_0/OakD-Lite/base_link'
        t3.child_frame_id = 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
        
        t3.transform.translation.x = 0.01233  # 相机在基座上的位置
        t3.transform.translation.y = -0.03
        t3.transform.translation.z = 0.01878
        
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t3)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()