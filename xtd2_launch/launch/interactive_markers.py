from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_marker')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose_3d', 10)
        self.server = InteractiveMarkerServer(self, "goal_marker")
        self.create_marker()

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'goal'
        int_marker.scale = 1.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = feedback.pose
        self.pub.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
