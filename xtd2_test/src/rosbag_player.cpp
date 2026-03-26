#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <chrono>

class RosbagPlayerWithSystemTime : public rclcpp::Node
{
public:
    explicit RosbagPlayerWithSystemTime(const std::string & bag_path)
        : Node("rosbag_player_with_system_time")
    {
        storage_options_.uri = bag_path;
        storage_options_.storage_id = "sqlite3";

        converter_options_.input_serialization_format = "";
        converter_options_.output_serialization_format = "";

        reader_.open(storage_options_, converter_options_);

        setupPublishers();

        RCLCPP_INFO(this->get_logger(), "Playing rosbag from: %s", bag_path.c_str());
        RCLCPP_INFO(this->get_logger(), "All message timestamps will be replaced with system time");

        playBag();
    }

private:
    void setupPublishers()
    {
        auto topics_info = reader_.get_all_topics_and_types();

        for (const auto & topic_info : topics_info) {
            const std::string & topic_name = topic_info.name;
            const std::string & topic_type = topic_info.type;

            try {
                auto publisher = this->create_generic_publisher(
                    topic_name,
                    topic_type,
                    rclcpp::QoS(rclcpp::KeepLast(10)));

                publishers_[topic_name] = publisher;
                topic_types_[topic_name] = topic_type;

                RCLCPP_INFO(this->get_logger(), "Set up publisher for topic: %s (%s)",
                           topic_name.c_str(), topic_type.c_str());
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create publisher for %s: %s",
                            topic_name.c_str(), e.what());
            }
        }
    }

    void playBag()
    {
        while (rclcpp::ok()) {
            rclcpp::Time start_time = this->now();
            rclcpp::Time bag_start_time(0, 0, RCL_ROS_TIME);
            
            bool first_msg = true;
            
            while (rclcpp::ok() && reader_.has_next()) {
                try {
                    auto msg = reader_.read_next();

                    auto it = publishers_.find(msg->topic_name);
                    if (it != publishers_.end()) {
                        auto publisher = it->second;
                        auto serialized_msg = msg->serialized_data;
                        
                        rclcpp::SerializedMessage serialized_message;
                        serialized_message.reserve(serialized_msg->buffer_length);
                        memcpy(serialized_message.get_rcl_serialized_message().buffer, 
                               serialized_msg->buffer, 
                               serialized_msg->buffer_length);
                        serialized_message.get_rcl_serialized_message().buffer_length = serialized_msg->buffer_length;
                        serialized_message.get_rcl_serialized_message().buffer_capacity = serialized_msg->buffer_length;

                        publisher->publish(serialized_message);
                    }

                    if (first_msg) {
                        bag_start_time = rclcpp::Time(msg->time_stamp);
                        first_msg = false;
                    } else {
                        rclcpp::Time current_bag_time(msg->time_stamp);
                        rclcpp::Duration time_diff = current_bag_time - bag_start_time;
                        
                        auto elapsed = this->now() - start_time;
                        auto sleep_time = time_diff - elapsed;
                        
                        if (sleep_time.nanoseconds() > 0) {
                            std::this_thread::sleep_for(
                                std::chrono::nanoseconds(sleep_time.nanoseconds()));
                        }
                    }

                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Finished playing rosbag, restarting loop...");
            
            reader_.close();
            reader_.open(storage_options_, converter_options_);
        }
    }

    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;
    rosbag2_cpp::readers::SequentialReader reader_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
    std::unordered_map<std::string, std::string> topic_types_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::string bag_path = "/home/cat/slam_data/livox_record/";
    if (argc > 1) {
        bag_path = argv[1];
    }

    auto player = std::make_shared<RosbagPlayerWithSystemTime>(bag_path);

    try {
        rclcpp::spin(player);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(player->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
