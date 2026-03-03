#!/usr/bin/env python3

import platform
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
from rclpy.parameter import Parameter

class RobotStatePublisher(Node):
    def __init__(self):
        # 检查主机名，设置默认namespace
        hostname = platform.node()
        if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':  
            default_namespace = '/x500_depth_0/'
        else:
            default_namespace = '/'
            
        super().__init__('robot_state_publisher_custom')
        
        # 获取namespace参数
        self.declare_parameter('namespace', default_namespace)
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # 发布joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 订阅odometry话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            [self.namespace, 'odometry'],
            self.odom_callback,
            10)
        
        # 定时发布joint states（所有joint都是固定的）
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info('Robot State Publisher started')
    
    def odom_callback(self, msg):
        # 这里可以处理odometry数据，如果需要动态joint可以在这里更新
        pass
    
    def publish_joint_states(self):
        # 发布joint states（所有joint都是固定的）
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # 所有joint名称
        joint_state.name = [
            'stereo_camera_joint',
            'imx214_camera_joint', 
            'propeller_1_joint',
            'propeller_2_joint',
            'propeller_3_joint',
            'propeller_4_joint'
        ]
        
        # 所有joint位置（都是固定的）
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    robot_state_publisher = RobotStatePublisher()
    
    try:
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()