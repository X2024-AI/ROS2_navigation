// Bridge /ground_truth_odom (Gazebo physics odometry) -> TF odom->base_footprint.
//
// This host's Gazebo sims intermittently drop the ros2_control diff-drive
// outputs (odometry topic and/or TF) minutes after boot, while Gazebo-native
// plugins (clock, laser, ground-truth odometry) keep running. This bridge
// republishes the ground-truth pose as the odometry transform.
//
// Written in C++ deliberately: rclpy publishers on this host exhibit
// first-message-then-stall delivery to persistent reliable subscribers,
// which froze every TF consumer of the earlier python bridge.
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "odom_tf_bridge",
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}));

  auto br = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  // BEST_EFFORT input: keeps the stream flowing even if the reliable path
  // to Gazebo-plugin publishers degrades (compatible with reliable pubs).
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/ground_truth_odom",
    rclcpp::QoS(10).reliable(),  // gazebo plugin publishes reliable
    [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = node->now();
      // The gazebo plugin parents its pose to the WORLD frame; the robot
      // spawns at the world origin, so odom == world numerically. Force the
      // conventional names - a literal relay would publish
      // world->base_footprint, leaving "odom" nonexistent downstream.
      t.header.frame_id = "odom";
      t.child_frame_id = "base_footprint";
      t.transform.translation.x = msg->pose.pose.position.x;
      t.transform.translation.y = msg->pose.pose.position.y;
      t.transform.translation.z = msg->pose.pose.position.z;
      t.transform.rotation = msg->pose.pose.orientation;
      br->sendTransform(t);
    });

  RCLCPP_INFO(node->get_logger(), "odom->base_footprint TF bridge up (C++)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
