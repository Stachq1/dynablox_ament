#ifndef POINTCLOUD_LISTENER_NODE_HPP
#define POINTCLOUD_LISTENER_NODE_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynablox/dynablox.h"

class PointCloudListenerNode : public rclcpp::Node {
public:
  PointCloudListenerNode(std::string node_name);

private:
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<dynablox::MapUpdater> map_updater_;

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_cloud_pub_;

  // Parameters
  std::string pcl_topic_;
  std::string dynamic_cloud_topic_;
  std::string dynablox_param_path_;
};

#endif // POINTCLOUD_LISTENER_NODE_HPP