import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import sys


class RosbagPlayerWithSystemTime(Node):
    def __init__(self, bag_path):
        super().__init__('rosbag_player_with_system_time')
        self.bag_path = bag_path
        self.topic_publishers = {}
        self.message_types = {}
        self.message_type_cache = {}
        
        self.storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        self.converter_options = rosbag2_py.ConverterOptions('', '')
        
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(self.storage_options, self.converter_options)
        
        self.setup_publishers()
        
        self.get_logger().info(f'Playing rosbag from: {bag_path}')
        self.get_logger().info('All message timestamps will be replaced with system time')
        
        self.play_bag()

    def setup_publishers(self):
        topic_types = self.reader.get_all_topics_and_types()
        qos_profile = QoSProfile(depth=10)
        
        for topic_info in topic_types:
            topic_name = topic_info.name
            topic_type = topic_info.type
            self.message_types[topic_name] = topic_type
            
            try:
                msg_type = get_message(topic_type)
                self.message_type_cache[topic_name] = msg_type
                self.topic_publishers[topic_name] = self.create_publisher(msg_type, topic_name, qos_profile)
                self.get_logger().info(f'Set up publisher for topic: {topic_name} ({topic_type})')
            except Exception as e:
                self.get_logger().error(f'Failed to create publisher for {topic_name}: {e}')

    def play_bag(self):
        clock = self.get_clock()
        while rclpy.ok():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic in self.topic_publishers:
                try:
                    msg_type = self.message_type_cache[topic]
                    msg = deserialize_message(data, msg_type)
                    
                    if hasattr(msg, 'header'):
                        msg.header.stamp = clock.now().to_msg()
                    
                    self.topic_publishers[topic].publish(msg)
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to publish message on {topic}: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    bag_path = '/home/cat/slam_data/livox_record/'
    
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    
    player = RosbagPlayerWithSystemTime(bag_path)
    
    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        player.get_logger().info('Shutting down...')
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
