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
        
        # 璁板綍涓婁竴娆＄殑浣嶇疆鍜屾柟鍚?        
        self.last_position = None
        self.last_yaw = 0.0
        
        self.create_marker()

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'goal'
        int_marker.scale = 1.0
        
        # 璁剧疆鍒濆浣嶇疆涓?0, 0, 2)
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 1.0
        int_marker.pose.orientation.w = 1.0

        # 鍒涘缓绠ごmarker鏉ユ樉绀烘柟鍚?        
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0  # 绠ご闀垮害
        marker.scale.y = 0.2  # 绠ご瀹藉害
        marker.scale.z = 0.2  # 绠ご楂樺害
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
            
        # 鑾峰彇褰撳墠浣嶇疆
        current_position = [
            feedback.pose.position.x,
            feedback.pose.position.y,
            feedback.pose.position.z
        ]
        
        # 璁＄畻yaw鏂瑰悜锛堝熀浜庣Щ鍔ㄦ柟鍚戯級
        if self.last_position is not None:
            # 璁＄畻绉诲姩鍚戦噺锛堟按骞抽潰锛?            
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            
            # 璁＄畻鍋忚埅瑙掞紙鍩轰簬绉诲姩鏂瑰悜锛?            
            if abs(dx) > 0.01 or abs(dy) > 0.01:  # 鏈夋樉钁楃Щ鍔?                
                yaw = math.atan2(dy, dx)  # 璁＄畻鏂瑰悜瑙?                
                # 鏍囧噯鍖栧亸鑸鍒癧-蟺, 蟺]
                if yaw > math.pi:
                    yaw -= 2 * math.pi
                elif yaw < -math.pi:
                    yaw += 2 * math.pi
                    
                self.last_yaw = yaw
            else:
                yaw = self.last_yaw  # 淇濇寔涓婁竴娆＄殑鍋忚埅瑙?        
        else:
            yaw = self.last_yaw  # 绗竴娆＄Щ鍔紝浣跨敤榛樿鍊?            
        # 鏇存柊涓婁竴娆′綅缃?        
        self.last_position = current_position
        
        # 鏇存柊marker鐨勬柟鍚戞樉绀?        
        self.update_marker_orientation(yaw)
        
        # 鍒涘缓骞跺彂甯冨甫鏈墆aw鏂瑰悜鐨刾ose
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = feedback.pose.position.x
        ps.pose.position.y = feedback.pose.position.y
        ps.pose.position.z = feedback.pose.position.z
        
        # 璁剧疆鍋忚埅瑙掓柟鍚?        
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pub.publish(ps)
        
        # 璁板綍璋冭瘯淇℃伅
        self.get_logger().info(f'馃幆 鐩爣鐐规洿鏂? ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f})')
        self.get_logger().info(f'馃Л 鍋忚埅瑙? {math.degrees(yaw):.1f}掳')

    def update_marker_orientation(self, yaw):
        # 鏇存柊marker鐨勬柟鍚戞樉绀?
        try:
            # 閲嶆柊鍒涘缓marker浠ユ洿鏂版柟鍚?            
            self.create_marker_with_orientation(yaw)
            self.get_logger().debug(f'馃攧 鏇存柊marker鏂瑰悜: {math.degrees(yaw):.1f}掳')
        except Exception as e:
            self.get_logger().warn(f'鈿狅笍 鏃犳硶鏇存柊marker鏂瑰悜: {str(e)}')

    def create_marker_with_orientation(self, yaw):
        # 鍒涘缓甯︽湁鐗瑰畾鏂瑰悜鐨刴arker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'goal'
        int_marker.scale = 1.0
        
        # 浣跨敤褰撳墠浣嶇疆锛堝鏋滃瓨鍦級鎴栧垵濮嬩綅缃?        
        if self.last_position is not None:
            int_marker.pose.position.x = self.last_position[0]
            int_marker.pose.position.y = self.last_position[1]
            int_marker.pose.position.z = self.last_position[2]
        else:
            # 璁剧疆鍒濆浣嶇疆涓?0, 0, 2)
            int_marker.pose.position.x = 0.0
            int_marker.pose.position.y = 0.0
            int_marker.pose.position.z = 2.0
        
        # 璁剧疆鏂瑰悜锛堝熀浜巠aw瑙掞級
        int_marker.pose.orientation.z = math.sin(yaw / 2.0)
        int_marker.pose.orientation.w = math.cos(yaw / 2.0)

        # 鍒涘缓绠ごmarker鏉ユ樉绀烘柟鍚?        
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0  # 绠ご闀垮害
        marker.scale.y = 0.2  # 绠ご瀹藉害
        marker.scale.z = 0.2  # 绠ご楂樺害
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

        # 鏇存柊server
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
