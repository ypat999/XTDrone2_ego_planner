#ifndef WEB_POINTCLOUD_BRIDGE_HPP
#define WEB_POINTCLOUD_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <map>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <nlohmann/json.hpp>

struct PointCloudConfig {
    std::string topic_name;
    std::string target_frame;
    double voxel_size;
    int max_points;
    std::string color_mode;
    bool enabled;
    double max_height;
    bool use_deduplication;
    std::unordered_set<int> sent_voxels;
};

class WebPointCloudBridge : public rclcpp::Node {
public:
    WebPointCloudBridge();
    ~WebPointCloudBridge() = default;

private:
    void configCallback(const std_msgs::msg::String::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic_name);
    void processConfig(const nlohmann::json& config);
    void addSubscription(const std::string& topic_name, const PointCloudConfig& config);
    void removeSubscription(const std::string& topic_name);
    sensor_msgs::msg::PointCloud2::SharedPtr processPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& input,
        PointCloudConfig& config);
    void publishProcessedCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
        const std::string& original_topic);

    std::map<std::string, PointCloudConfig> configs_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
};

#endif // WEB_POINTCLOUD_BRIDGE_HPP
