#include <cmath>
#include <string>

#include "wr_bringup/DockApproach.hpp"

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

  slowed_ = false;          // fresh approach: full speed until the slow zone
  last_logged_dist_ = -1.0;
}

void
DockApproach::on_wait_for_result()
{
  if (!pose_received_) {
    return;
  }

  const double dist = distance_to_goal();

  // progress trace at ~5 cm resolution (called every BT tick)
  if (last_logged_dist_ < 0.0 || std::fabs(last_logged_dist_ - dist) > 0.05) {
    last_logged_dist_ = dist;
    RCLCPP_INFO(node_->get_logger(), "dock approach: %.2f m from dock", dist);
  }

  if (slowed_) {
    return;
  }
  double threshold = 1.0, slow_speed = 0.25;
  getInput("distance_threshold", threshold);
  getInput("slow_speed", slow_speed);
  if (dist < threshold) {
    slowed_ = true;
    if (slow_speed > 0.0) {
      // optional: absolute SpeedLimit. NOTE nav2's absolute limit also clamps
      // wz, which can dither MPPI's terminal phase off-axis - leave 0 to
      // slow down via a cautious controller plugin instead.
      limit_speed(slow_speed, false);
      RCLCPP_INFO(
        node_->get_logger(),
        "dock approach: %.2f m (< %.2f) -> speed limited to %.2f m/s",
        dist, threshold, slow_speed);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "dock approach: %.2f m (< %.2f) -> creep zone (slow controller)",
        dist, threshold);
    }
  }
}

BT::NodeStatus
DockApproach::on_success()
{
  // restore: absolute set (idempotent, no stacking). vx is restored fully;
  // wz ends at the same value, which is adequate for subsequent navigation.
  limit_speed(0.5, false);
  RCLCPP_INFO(node_->get_logger(), "dock navigation succeeded (%.2f m from dock)",
    pose_received_ ? distance_to_goal() : -1.0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
DockApproach::on_aborted()
{
  limit_speed(0.5, false);
  RCLCPP_WARN(node_->get_logger(), "dock navigation ABORTED (%.2f m from dock)",
    pose_received_ ? distance_to_goal() : -1.0);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus
DockApproach::on_cancelled()
{
  limit_speed(0.5, false);
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
