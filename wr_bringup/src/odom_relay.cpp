// Relay /ground_truth_odom -> /odom (C++).
//
// nav2's behavior server reads a topic literally named "odom"; this host's
// ros2_control diff-drive odometry dies mid-session, and topic_tools' python
// relay stalls after its first message (rclpy<->Gazebo malady). A C++
// relay on the Gazebo ground-truth stream is the reliable source.
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("odom_relay");

  auto pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10));

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/ground_truth_odom", rclcpp::QoS(10),
    [pub](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      pub->publish(*msg);
    });

  RCLCPP_INFO(node->get_logger(), "ground_truth -> /odom relay up (C++)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
