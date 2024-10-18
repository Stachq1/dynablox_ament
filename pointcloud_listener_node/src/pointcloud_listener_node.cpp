#include "pointcloud_listener_node/pointcloud_listener_node.hpp"

PointCloudListenerNode::PointCloudListenerNode(std::string node_name) : rclcpp::Node(node_name) {
  // Declare the 'source_pcl_topic' parameter with a default value
  this->declare_parameter<std::string>("source_pcl_topic", "/cloud_registered_body");

  // Declare the 'dynamic_pcl_topic' parameter with a default value
  this->declare_parameter<std::string>("dynamic_pcl_topic", "/cloud_dynamic_body");

  // Declare the 'pcl_frame' parameter with a default value
  this->declare_parameter<std::string>("pcl_frame", "body_lidar");

  // Declare the 'dynablox_param_path' parameter with a default value
  this->declare_parameter<std::string>("dynablox_param_path", "assets/config.yaml");

  // Retrieve the parameters from the node's parameter server
  if (!this->get_parameter("source_pcl_topic", source_pcl_topic_)) {
    RCLCPP_WARN(this->get_logger(), "source_pcl_topic not found. Using default value.");
  }

  if (!this->get_parameter("dynablox_param_path", dynablox_param_path_)) {
    RCLCPP_WARN(this->get_logger(), "dynablox_param_path not found. Using default value.");
  }

  if(!this->get_parameter("pcl_frame", pcl_frame_)) {
    RCLCPP_WARN(this->get_logger(), "pcl_frame not found. Using default value.");
  }

  if (!this->get_parameter("dynamic_pcl_topic", dynamic_pcl_topic_)) {
    RCLCPP_WARN(this->get_logger(), "dynamic_pcl_topic not found. Using default value.");
  }

  // Create a MapUpdater instance
  map_updater_ = std::make_shared<dynablox::MapUpdater>(dynablox_param_path_);

  // Initialize the ROS communication
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      source_pcl_topic_, 10, std::bind(&PointCloudListenerNode::pclCallback, this, std::placeholders::_1));
  dynamic_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(dynamic_pcl_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", source_pcl_topic_.c_str());
}

void PointCloudListenerNode::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert ROS point cloud to PCL point cloud
  pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *pcl_cloud);

  // Advance the map updater with the new point cloud
  map_updater_->run(pcl_cloud);

  // Get current dynamic points
  sensor_msgs::msg::PointCloud2 ros_dynamic_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_cloud = map_updater_->getDynamicCloud();

  // Print the size of the dynamic point cloud from Dynablox
  RCLCPP_WARN(this->get_logger(), "Dynamic point cloud has %zu points", dynamic_cloud->size());

  pcl::toROSMsg(*dynamic_cloud, ros_dynamic_cloud);
  ros_dynamic_cloud.header.stamp = this->now();
  ros_dynamic_cloud.header.frame_id = pcl_frame_;

  // Publish them
  dynamic_cloud_pub_->publish(ros_dynamic_cloud);

  // Reset the dynamic pointcloud inside Dynablox
  map_updater_->resetDynamicCloud();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudListenerNode>("pointcloud_listener_node"));
    rclcpp::shutdown();
    return 0;
}
