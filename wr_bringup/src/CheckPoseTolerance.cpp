#include <cmath>
#include <string>

#include "wr_bringup/CheckPoseTolerance.hpp"

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wr_bringup
{

DockFrameError compute_dock_frame_error(
  const geometry_msgs::msg::PoseStamped & dock,
  tf2_ros::Buffer & tf_buffer)
{
  DockFrameError e;
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    (void)ex;
    return e;   // valid stays false
  }

  const double dx = t.transform.translation.x - dock.pose.position.x;
  const double dy = t.transform.translation.y - dock.pose.position.y;
  const double dock_yaw = tf2::getYaw(dock.pose.orientation);
  const double robot_yaw = tf2::getYaw(t.transform.rotation);

  e.longitudinal = dx * std::cos(dock_yaw) + dy * std::sin(dock_yaw);
  e.lateral = -dx * std::sin(dock_yaw) + dy * std::cos(dock_yaw);
  e.yaw = std::atan2(
    std::sin(robot_yaw - dock_yaw), std::cos(robot_yaw - dock_yaw));
  e.valid = true;
  return e;
}

CheckPoseTolerance::CheckPoseTolerance(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  // Prefer the tree-wide shared TF buffer (set by parking_bt_main): it keeps
  // the last map->odom cached even while AMCL is idle at standstill. Fall
  // back to a private buffer when this node runs standalone.
  if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_) ||
    !tf_buffer_)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
  }
}

BT::NodeStatus
CheckPoseTolerance::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped dock;
  getInput("dock", dock);
  double pos_tol = 0.02, yaw_tol = 0.00873;
  getInput("pos_tol", pos_tol);
  getInput("yaw_tol", yaw_tol);

  const DockFrameError e = compute_dock_frame_error(dock, *tf_buffer_);
  if (!e.valid) {
    RCLCPP_WARN(node->get_logger(), "tolerance check: TF unavailable");
    return BT::NodeStatus::FAILURE;
  }

  const double pos_err = std::hypot(e.longitudinal, e.lateral);
  RCLCPP_INFO(
    node->get_logger(),
    "tolerance check: pos %.4f m (lon %+.3f, lat %+.3f, tol %.3f), yaw %+.4f rad (tol %.4f)",
    pos_err, e.longitudinal, e.lateral, pos_tol, e.yaw, yaw_tol);

  const bool ok = pos_err <= pos_tol && std::fabs(e.yaw) <= yaw_tol;
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::CheckPoseTolerance>("CheckPoseTolerance");
}
