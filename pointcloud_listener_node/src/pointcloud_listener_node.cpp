#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynablox/dynablox.h"

class PointCloudListenerNode : public rclcpp::Node {
public:
  PointCloudListenerNode() {
    // Declare the 'pcl_topic' parameter with a default value
    this->declare_parameter<std::string>("pcl_topic", "/default_point_cloud_topic");

      // Retrieve the parameter from the node's parameter server
    if (!this->get_parameter("pcl_topic", pcl_topic_)) {
      RCLCPP_WARN(this->get_logger(), "Topic name parameter not found. Using default value.");
    }

    pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pcl_topic_, 10, std::bind(&PointCloudListenerNode::pclCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", pcl_topic_.c_str());
  }

private:
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      run(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSub_;
  std::string pcl_topic_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudListenerNode>());
    rclcpp::shutdown();
    return 0;
}
