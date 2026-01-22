#!/usr/bin/env python3
"""
测试轨迹服务器的调试输出
监听轨迹服务器的调试信息
"""

import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline
from geometry_msgs.msg import Pose
import time
import numpy as np

class TrajServerDebugMonitor(Node):
    def __init__(self):
        super().__init__('traj_server_debug_monitor')
        
        # 订阅轨迹命令输出
        self.cmd_sub = self.create_subscription(
            Pose, 
            '/xtdrone2/planning/cmd_pose_local_ned', 
            self.cmd_callback, 
            10
        )
        
        # 发布测试轨迹
        self.traj_pub = self.create_publisher(Bspline, '/planning/bspline', 10)
        
        self.get_logger().info("🧪 轨迹服务器调试监控已启动")
        self.get_logger().info("📡 监听话题: /xtdrone2/planning/cmd_pose_local_ned")
        self.get_logger().info("📤 发布测试轨迹到: /planning/bspline")
        
    def cmd_callback(self, msg):
        """接收轨迹命令输出"""
        self.get_logger().info(f"📍 收到轨迹命令:")
        self.get_logger().info(f"   位置: ({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})")
        self.get_logger().info(f"   四元数: [{msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f}]")
        
        # 从四元数计算偏航角
        import math
        yaw = 2 * math.atan2(msg.orientation.z, msg.orientation.w)
        self.get_logger().info(f"   偏航角: {math.degrees(yaw):.1f}°")
        
    def publish_test_trajectory(self):
        """发布测试轨迹"""
        # 创建简单的B样条轨迹
        bspline = Bspline()
        bspline.header.stamp = self.get_clock().now().to_msg()
        bspline.header.frame_id = "world"
        bspline.traj_id = 1
        bspline.polynomial_order = 5
        
        # 设置时间参数
        bspline.start_time.sec = int(time.time())
        bspline.start_time.nanosec = 0
        bspline.traj_time = 10.0  # 10秒轨迹
        
        # 添加位置控制点（简单的直线轨迹）
        # 从(0,0,0)到(5,3,2)
        waypoints = [
            (0.0, 0.0, 0.0),
            (1.0, 0.5, 0.4),
            (2.0, 1.0, 0.8),
            (3.0, 1.5, 1.2),
            (4.0, 2.0, 1.6),
            (5.0, 3.0, 2.0),
        ]
        
        for i, (x, y, z) in enumerate(waypoints):
            point = Pose()
            point.position.x = x
            point.position.y = y
            point.position.z = z
            bspline.pos_pts.append(point)
            
        bspline.knots.append(0.0)  # 起始节点
        for i in range(len(waypoints) + 5):  # B样条节点
            bspline.knots.append(float(i))
            
        self.traj_pub.publish(bspline)
        self.get_logger().info("🚀 发布测试轨迹")
        
    def monitor_rosout(self):
        """监控ROS日志输出"""
        import subprocess
        import threading
        
        def log_monitor():
            try:
                # 监听轨迹服务器的日志输出
                process = subprocess.Popen(
                    ['ros2', 'topic', 'echo', '/rosout'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                
                self.get_logger().info("📋 开始监控ROS日志...")
                
                for line in iter(process.stdout.readline, ''):
                    if 'traj_server' in line and ('dir向量' in line or 'atan2' in line or '坐标转换' in line):
                        self.get_logger().info(f"📝 调试信息: {line.strip()}")
                        
            except Exception as e:
                self.get_logger().error(f"日志监控错误: {str(e)}")
                
        # 启动日志监控线程
        log_thread = threading.Thread(target=log_monitor)
        log_thread.daemon = True
        log_thread.start()

def main():
    rclpy.init()
    node = TrajServerDebugMonitor()
    
    # 启动日志监控
    node.monitor_rosout()
    
    # 等待一段时间让系统启动
    time.sleep(2.0)
    
    # 发布测试轨迹
    node.publish_test_trajectory()
    
    # 保持运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()