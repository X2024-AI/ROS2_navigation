#include <cmath>
#include <string>

#include "wr_bringup/DockApproach.hpp"

#include "nav2_msgs/msg/speed_limit.hpp"

namespace wr_bringup
{

DockApproach::DockApproach(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: wr_bringup::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 1,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {on_pose(msg);});
  speed_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 1);
}

void
DockApproach::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = *msg;
  pose_received_ = true;
}

double
DockApproach::distance_to_goal() const
{
  const double dx = last_pose_.pose.pose.position.x - goal_pose_.pose.position.x;
  const double dy = last_pose_.pose.pose.position.y - goal_pose_.pose.position.y;
  return std::hypot(dx, dy);
}

void
DockApproach::limit_speed(double limit, bool percentage)
{
  nav2_msgs::msg::SpeedLimit msg;
  msg.percentage = percentage;
  msg.speed_limit = static_cast<float>(limit);
  speed_pub_->publish(msg);
}

void
DockApproach::on_tick()
{
  getInput("goal", goal_pose_);
  goal_.pose = goal_pose_;

  std::string bt_xml;
  getInput("bt_xml", bt_xml);
  goal_.behavior_tree = bt_xml;

  slowed_ = false;   // fresh approach: full speed until the slow zone
}

void
DockApproach::on_wait_for_result()
{
  if (slowed_ || !pose_received_) {
    return;
  }

  double threshold = 1.5, slow_speed = 0.15;
  getInput("distance_threshold", threshold);
  getInput("slow_speed", slow_speed);

  const double dist = distance_to_goal();
  if (dist < threshold) {
    limit_speed(slow_speed, false);   // absolute m/s
    slowed_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "dock approach: %.2f m from dock (< %.2f) -> speed limited to %.2f m/s",
      dist, threshold, slow_speed);
  }
}

BT::NodeStatus
DockApproach::on_success()
{
  limit_speed(100.0, true);   // restore full speed
  RCLCPP_INFO(node_->get_logger(), "dock navigation succeeded (slow=%s)",
    slowed_ ? "yes" : "no");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
DockApproach::on_aborted()
{
  limit_speed(100.0, true);
  RCLCPP_WARN(node_->get_logger(), "dock navigation ABORTED (slow=%s, last dist %.2f m)",
    slowed_ ? "yes" : "no", pose_received_ ? distance_to_goal() : -1.0);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus
DockApproach::on_cancelled()
{
  limit_speed(100.0, true);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<wr_bringup::DockApproach>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<wr_bringup::DockApproach>("DockApproach", builder);
}
