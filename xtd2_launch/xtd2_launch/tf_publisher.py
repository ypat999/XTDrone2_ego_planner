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
        # 发布base_link到odom的TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'x500_depth_0/base_link'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
    
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
        
        # 发布base_link到相机的TF
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'x500_depth_0/base_link'
        t2.child_frame_id = 'x500_depth_0/StereoOV7251'
        
        t2.transform.translation.x = 0.1  # 相机在飞机前方10cm
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.05  # 相机在飞机上方5cm
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)

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