#include "web_pointcloud_bridge.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

WebPointCloudBridge::WebPointCloudBridge()
    : Node("web_pointcloud_bridge")
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    config_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/web_pointcloud/config",
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
        std::bind(&WebPointCloudBridge::configCallback, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/web_pointcloud/status",
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort));

    PointCloudConfig default_config;
    default_config.topic_name = "/grid_map/occupancy_inflate";
    default_config.target_frame = "map";
    default_config.voxel_size = 0.5;
    default_config.max_points = 2000;
    default_config.color_mode = "z-axis";
    default_config.enabled = true;
    default_config.max_height = 0.0;
    addSubscription(default_config.topic_name, default_config);

    RCLCPP_INFO(this->get_logger(), "Web PointCloud Bridge initialized");
}

void WebPointCloudBridge::configCallback(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        auto config = nlohmann::json::parse(msg->data);
        processConfig(config);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse config: %s", e.what());
    }
}

void WebPointCloudBridge::processConfig(const nlohmann::json& config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (config.contains("action")) {
        std::string action = config["action"];
        
        if (action == "add" && config.contains("topics")) {
            for (const auto& topic_config : config["topics"]) {
                PointCloudConfig cfg;
                cfg.topic_name = topic_config.value("topic_name", "");
                cfg.target_frame = topic_config.value("target_frame", "map");
                cfg.voxel_size = topic_config.value("voxel_size", 0.1);
                cfg.max_points = topic_config.value("max_points", 50000);
                cfg.color_mode = topic_config.value("color_mode", "z-axis");
                cfg.enabled = topic_config.value("enabled", true);
                cfg.max_height = topic_config.value("max_height", 0.0);
                cfg.use_deduplication = (cfg.topic_name == "/grid_map/occupancy_inflate");
                cfg.sent_voxels.clear();

                if (!cfg.topic_name.empty()) {
                    addSubscription(cfg.topic_name, cfg);
                }
            }
        } else if (action == "remove" && config.contains("topic_name")) {
            removeSubscription(config["topic_name"]);
        } else if (action == "update" && config.contains("topic_name")) {
            std::string topic_name = config["topic_name"];
            if (configs_.count(topic_name)) {
                PointCloudConfig& cfg = configs_[topic_name];
                bool params_changed = false;
                if (config.contains("target_frame")) {
                    cfg.target_frame = config["target_frame"];
                    params_changed = true;
                }
                if (config.contains("voxel_size")) {
                    cfg.voxel_size = config["voxel_size"];
                    params_changed = true;
                }
                if (config.contains("max_points")) {
                    cfg.max_points = config["max_points"];
                    params_changed = true;
                }
                if (config.contains("color_mode")) {
                    cfg.color_mode = config["color_mode"];
                }
                if (config.contains("enabled")) {
                    cfg.enabled = config["enabled"];
                }
                if (config.contains("max_height")) {
                    cfg.max_height = config["max_height"];
                    params_changed = true;
                }
                if (params_changed && cfg.use_deduplication) {
                    cfg.sent_voxels.clear();
                    RCLCPP_INFO(this->get_logger(), "Cleared deduplication cache for %s due to parameter change", topic_name.c_str());
                }
            }
        } else if (action == "list") {
            nlohmann::json status;
            status["type"] = "topic_list";
            status["topics"] = nlohmann::json::array();
            for (const auto& [name, cfg] : configs_) {
                nlohmann::json topic_info;
                topic_info["topic_name"] = name;
                topic_info["target_frame"] = cfg.target_frame;
                topic_info["voxel_size"] = cfg.voxel_size;
                topic_info["max_points"] = cfg.max_points;
                topic_info["color_mode"] = cfg.color_mode;
                topic_info["enabled"] = cfg.enabled;
                topic_info["max_height"] = cfg.max_height;
                status["topics"].push_back(topic_info);
            }
            auto status_msg = std_msgs::msg::String();
            status_msg.data = status.dump();
            status_pub_->publish(status_msg);
        }
    }
}

void WebPointCloudBridge::addSubscription(const std::string& topic_name, const PointCloudConfig& config)
{
    if (subscriptions_.count(topic_name)) {
        RCLCPP_WARN(this->get_logger(), "Subscription for %s already exists, updating config", topic_name.c_str());
        configs_[topic_name] = config;
        
        if (!publishers_.count(topic_name)) {
            std::string output_topic = topic_name + "/web_processed";
            auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                output_topic,
                rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort));
            publishers_[topic_name] = pub;
            RCLCPP_INFO(this->get_logger(), "Created publisher for existing subscription: %s", output_topic.c_str());
        }
        return;
    }

    configs_[topic_name] = config;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.qos_overriding_options = rclcpp::QosOverridingOptions({
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Reliability,
        rclcpp::QosPolicyKind::Durability
    });

    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name,
        rclcpp::QoS(10).best_effort(),
        [this, topic_name](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pointCloudCallback(msg, topic_name);
        },
        sub_options);

    subscriptions_[topic_name] = sub;

    std::string output_topic = topic_name + "/web_processed";
    auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic,
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort));
    publishers_[topic_name] = pub;

    RCLCPP_INFO(this->get_logger(), "Added subscription for %s, output: %s", 
                topic_name.c_str(), output_topic.c_str());
}

void WebPointCloudBridge::removeSubscription(const std::string& topic_name)
{
    if (subscriptions_.count(topic_name)) {
        subscriptions_.erase(topic_name);
        publishers_.erase(topic_name);
        configs_.erase(topic_name);
        RCLCPP_INFO(this->get_logger(), "Removed subscription for %s", topic_name.c_str());
    }
}

void WebPointCloudBridge::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!configs_.count(topic_name) || !configs_[topic_name].enabled) {
        return;
    }

    auto& config = configs_[topic_name];
    auto processed = processPointCloud(msg, config);
    
    if (processed) {
        publishProcessedCloud(processed, topic_name);
    }
}

sensor_msgs::msg::PointCloud2::SharedPtr WebPointCloudBridge::processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input,
    PointCloudConfig& config)
{
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud;
    
    std::string source_frame = input->header.frame_id;
    if (!source_frame.empty() && source_frame[0] == '/') {
        source_frame = source_frame.substr(1);
    }

    if (!config.target_frame.empty() && source_frame != config.target_frame) {
        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
                config.target_frame, source_frame,
                input->header.stamp, rclcpp::Duration::from_seconds(0.1));
            
            transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            tf2::doTransform(*input, *transformed_cloud, transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TF transform failed for %s -> %s: %s",
                source_frame.c_str(), config.target_frame.c_str(), ex.what());
            transformed_cloud = input;
        }
    } else {
        transformed_cloud = input;
    }

    bool has_intensity = false;
    for (const auto& field : transformed_cloud->fields) {
        if (field.name == "intensity") {
            has_intensity = true;
            break;
        }
    }

    auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();

    if (has_intensity) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*transformed_cloud, *pcl_cloud);

        if (config.use_deduplication && config.voxel_size > 0.0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr deduplicated(new pcl::PointCloud<pcl::PointXYZI>);
            double voxel_size = config.voxel_size;
            int grid_x = static_cast<int>(std::ceil(200.0 / voxel_size));
            int grid_y = grid_x;
            double max_z = config.max_height > 0.0 ? config.max_height * 2.0 : 100.0;
            int grid_z = static_cast<int>(std::ceil(max_z / voxel_size));
            
            for (const auto& point : pcl_cloud->points) {
                int ix = static_cast<int>(std::floor(point.x / voxel_size));
                int iy = static_cast<int>(std::floor(point.y / voxel_size));
                int iz = static_cast<int>(std::floor(point.z / voxel_size));
                
                if (ix < 0 || ix >= grid_x || iy < 0 || iy >= grid_y || iz < 0 || iz >= grid_z) {
                    continue;
                }
                
                int idx = ix + iy * grid_x + iz * grid_x * grid_y;
                
                if (config.sent_voxels.find(idx) == config.sent_voxels.end()) {
                    config.sent_voxels.insert(idx);
                    deduplicated->points.push_back(point);
                }
            }
            deduplicated->width = deduplicated->points.size();
            deduplicated->height = 1;
            deduplicated->is_dense = pcl_cloud->is_dense;
            pcl_cloud = deduplicated;
        } else if (config.voxel_size > 0.0 && pcl_cloud->size() > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(pcl_cloud);
            voxel_filter.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);
            voxel_filter.filter(*filtered);
            pcl_cloud = filtered;
        }

        if (config.max_height > 0.0 && pcl_cloud->size() > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PassThrough<pcl::PointXYZI> pass_filter;
            pass_filter.setInputCloud(pcl_cloud);
            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(0.0, config.max_height);
            pass_filter.filter(*height_filtered);
            pcl_cloud = height_filtered;
        }

        if (config.max_points > 0 && static_cast<int>(pcl_cloud->size()) > config.max_points) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZI>);
            float ratio = static_cast<float>(config.max_points) / pcl_cloud->size();
            for (const auto& point : pcl_cloud->points) {
                if (static_cast<float>(rand()) / RAND_MAX < ratio) {
                    sampled->points.push_back(point);
                }
            }
            sampled->width = sampled->points.size();
            sampled->height = 1;
            sampled->is_dense = pcl_cloud->is_dense;
            pcl_cloud = sampled;
        }

        pcl::toROSMsg(*pcl_cloud, *output);
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*transformed_cloud, *pcl_cloud);

        if (config.use_deduplication && config.voxel_size > 0.0) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr deduplicated(new pcl::PointCloud<pcl::PointXYZ>);
            double voxel_size = config.voxel_size;
            int grid_x = static_cast<int>(std::ceil(200.0 / voxel_size));
            int grid_y = grid_x;
            double max_z = config.max_height > 0.0 ? config.max_height * 2.0 : 100.0;
            int grid_z = static_cast<int>(std::ceil(max_z / voxel_size));
            
            for (const auto& point : pcl_cloud->points) {
                int ix = static_cast<int>(std::floor(point.x / voxel_size));
                int iy = static_cast<int>(std::floor(point.y / voxel_size));
                int iz = static_cast<int>(std::floor(point.z / voxel_size));
                
                if (ix < 0 || ix >= grid_x || iy < 0 || iy >= grid_y || iz < 0 || iz >= grid_z) {
                    continue;
                }
                
                int idx = ix + iy * grid_x + iz * grid_x * grid_y;
                
                if (config.sent_voxels.find(idx) == config.sent_voxels.end()) {
                    config.sent_voxels.insert(idx);
                    deduplicated->points.push_back(point);
                }
            }
            deduplicated->width = deduplicated->points.size();
            deduplicated->height = 1;
            deduplicated->is_dense = pcl_cloud->is_dense;
            pcl_cloud = deduplicated;
        } else if (config.voxel_size > 0.0 && pcl_cloud->size() > 0) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(pcl_cloud);
            voxel_filter.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);
            voxel_filter.filter(*filtered);
            pcl_cloud = filtered;
        }

        if (config.max_height > 0.0 && pcl_cloud->size() > 0) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass_filter;
            pass_filter.setInputCloud(pcl_cloud);
            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(0.0, config.max_height);
            pass_filter.filter(*height_filtered);
            pcl_cloud = height_filtered;
        }

        if (config.max_points > 0 && static_cast<int>(pcl_cloud->size()) > config.max_points) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
            float ratio = static_cast<float>(config.max_points) / pcl_cloud->size();
            for (const auto& point : pcl_cloud->points) {
                if (static_cast<float>(rand()) / RAND_MAX < ratio) {
                    sampled->points.push_back(point);
                }
            }
            sampled->width = sampled->points.size();
            sampled->height = 1;
            sampled->is_dense = pcl_cloud->is_dense;
            pcl_cloud = sampled;
        }

        pcl::toROSMsg(*pcl_cloud, *output);
    }

    output->header = transformed_cloud->header;
    output->header.frame_id = config.target_frame;

    return output;
}

void WebPointCloudBridge::publishProcessedCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
    const std::string& original_topic)
{
    if (publishers_.count(original_topic)) {
        publishers_[original_topic]->publish(*cloud);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebPointCloudBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
