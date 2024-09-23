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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  std::string pcl_topic_;
};

#endif // POINTCLOUD_LISTENER_NODE_HPP