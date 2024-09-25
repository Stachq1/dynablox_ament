#include "pointcloud_listener_node/pointcloud_listener_node.hpp"

PointCloudListenerNode::PointCloudListenerNode(std::string node_name) : rclcpp::Node(node_name) {
  // Declare the 'pcl_topic' parameter with a default value
  this->declare_parameter<std::string>("pcl_topic", "/default_point_cloud_topic");

  // Declare the 'dynablox_param_path' parameter with a default value
  this->declare_parameter<std::string>("dynablox_param_path", "assets/config.yaml");

  // Retrieve the parameters from the node's parameter server
  if (!this->get_parameter("pcl_topic", pcl_topic_)) {
    RCLCPP_WARN(this->get_logger(), "pcl_topic not found. Using default value.");
  }

  if (!this->get_parameter("dynablox_param_path", dynablox_param_path_)) {
    RCLCPP_WARN(this->get_logger(), "dynablox_param_path not found. Using default value.");
  }

  // Create a MapUpdater instance
  map_updater_ = std::make_shared<dynablox::MapUpdater>(dynablox_param_path_);

  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pcl_topic_, 10, std::bind(&PointCloudListenerNode::pclCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", pcl_topic_.c_str());
}

void PointCloudListenerNode::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert ROS point cloud to PCL point cloud
  pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *pcl_cloud);

  // Advance the map updater with the new point cloud
  map_updater_->run(pcl_cloud);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudListenerNode>("pointcloud_listener_node"));
    rclcpp::shutdown();
    return 0;
}
