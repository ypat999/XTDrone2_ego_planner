from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
import math

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_marker')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose_3d', 10)
        self.server = InteractiveMarkerServer(self, "goal_marker")
        
        # 记录上一次的位置和方向
        self.last_position = None
        self.last_yaw = 0.0
        
        self.create_marker()

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'goal'
        int_marker.scale = 1.0
        
        # 设置初始位置为(0, 0, 2)
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 1.0
        int_marker.pose.orientation.w = 1.0

        # 创建箭头marker来显示方向
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0  # 箭头长度
        marker.scale.y = 0.2  # 箭头宽度
        marker.scale.z = 0.2  # 箭头高度
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.name = 'move_3d'
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True

        control.markers.append(marker)

        int_marker.controls.append(control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
            
        # 获取当前位置
        current_position = [
            feedback.pose.position.x,
            feedback.pose.position.y,
            feedback.pose.position.z
        ]
        
        # 计算yaw方向（基于移动方向）
        if self.last_position is not None:
            # 计算移动向量（水平面）
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            
            # 计算偏航角（基于移动方向）
            if abs(dx) > 0.01 or abs(dy) > 0.01:  # 有显著移动
                yaw = math.atan2(dy, dx)  # 计算方向角
                
                # 标准化偏航角到[-π, π]
                if yaw > math.pi:
                    yaw -= 2 * math.pi
                elif yaw < -math.pi:
                    yaw += 2 * math.pi
                    
                self.last_yaw = yaw
            else:
                yaw = self.last_yaw  # 保持上一次的偏航角
        else:
            yaw = self.last_yaw  # 第一次移动，使用默认值
            
        # 更新上一次位置
        self.last_position = current_position
        
        # 更新marker的方向显示
        self.update_marker_orientation(yaw)
        
        # 创建并发布带有yaw方向的pose
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = feedback.pose.position.x
        ps.pose.position.y = feedback.pose.position.y
        ps.pose.position.z = feedback.pose.position.z
        
        # 设置偏航角方向
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pub.publish(ps)
        
        # 记录调试信息
        self.get_logger().info(f'🎯 目标点更新: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f})')
        self.get_logger().info(f'🧭 偏航角: {math.degrees(yaw):.1f}°')

    def update_marker_orientation(self, yaw):
        """更新marker的方向显示"""
        try:
            # 重新创建marker以更新方向
            self.create_marker_with_orientation(yaw)
            self.get_logger().debug(f'🔄 更新marker方向: {math.degrees(yaw):.1f}°')
        except Exception as e:
            self.get_logger().warn(f'⚠️ 无法更新marker方向: {str(e)}')

    def create_marker_with_orientation(self, yaw):
        """创建带有特定方向的marker"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'goal'
        int_marker.scale = 1.0
        
        # 使用当前位置（如果存在）或初始位置
        if self.last_position is not None:
            int_marker.pose.position.x = self.last_position[0]
            int_marker.pose.position.y = self.last_position[1]
            int_marker.pose.position.z = self.last_position[2]
        else:
            # 设置初始位置为(0, 0, 2)
            int_marker.pose.position.x = 0.0
            int_marker.pose.position.y = 0.0
            int_marker.pose.position.z = 2.0
        
        # 设置方向（基于yaw角）
        int_marker.pose.orientation.z = math.sin(yaw / 2.0)
        int_marker.pose.orientation.w = math.cos(yaw / 2.0)

        # 创建箭头marker来显示方向
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0  # 箭头长度
        marker.scale.y = 0.2  # 箭头宽度
        marker.scale.z = 0.2  # 箭头高度
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.name = 'move_3d'
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True

        control.markers.append(marker)
        int_marker.controls.append(control)

        # 更新server
        self.server.erase('goal')
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
