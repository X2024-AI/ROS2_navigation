// Relay an odometry topic -> /odom (C++).
//
// nav2's behavior server reads a topic literally named "odom", and the M6
// parking nodes measure from /odom as well. topic_tools' python relay
// stalls after its first message for persistent C++ subscribers (the
// rclpy<->rclcpp malady on this host - measured 2026-10-20: freezer saw a
// frozen odom pose while fresh subscribers kept receiving), so the relay
// must be C++.
//
// Source topic is a parameter, defaulting to the ros2_control wheel
// odometry (single-source topology - see bringup_m6.sh [2/5]). Pass
//   -p source:=/ground_truth_odom
// for the fallback when wheel odometry is dead.
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("odom_relay");

  std::string source = "/mobile_base_controller/odom";
  node->declare_parameter<std::string>("source", source);
  node->get_parameter<std::string>("source", source);

  auto pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10));

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    source, rclcpp::QoS(10),
    [pub](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      pub->publish(*msg);
    });

  RCLCPP_INFO(node->get_logger(), "%s -> /odom relay up (C++)", source.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
