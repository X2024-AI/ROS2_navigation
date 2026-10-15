#include <cmath>
#include <string>

#include "wr_bringup/CheckPoseTolerance.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wr_bringup
{

DockFrameError compute_dock_frame_error(
  const geometry_msgs::msg::PoseStamped & dock,
  const tf2::Transform & map_base)
{
  DockFrameError e;

  const double dx = map_base.getOrigin().x() - dock.pose.position.x;
  const double dy = map_base.getOrigin().y() - dock.pose.position.y;
  const double dock_yaw = tf2::getYaw(dock.pose.orientation);
  const double robot_yaw = tf2::getYaw(map_base.getRotation());

  e.longitudinal = dx * std::cos(dock_yaw) + dy * std::sin(dock_yaw);
  e.lateral = -dx * std::sin(dock_yaw) + dy * std::cos(dock_yaw);
  e.yaw = std::atan2(
    std::sin(robot_yaw - dock_yaw), std::cos(robot_yaw - dock_yaw));
  e.valid = true;
  return e;
}

void attach_odom_source(
  const rclcpp::Node::SharedPtr & node,
  MapOdomFreezer & freezer)
{
  node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10),
    [&freezer](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      tf2::Transform ob;
      tf2::fromMsg(msg->pose.pose, ob);
      freezer.odom_base_ = ob;
      freezer.odom_received_ = true;
    });
}

bool compose_map_base(
  const BT::NodeConfiguration & config,
  tf2_ros::Buffer & tf_buffer,
  MapOdomFreezer & freezer,
  tf2::Transform & map_base_out)
{
  bool frozen = false;
  config.blackboard->get("odom_frozen", frozen);

  if (!frozen || !freezer.cached_) {
    // refresh the cache: latest AMCL map->odom (one-shot TF use)
    try {
      auto m = tf_buffer.lookupTransform("map", "odom", tf2::TimePointZero);
      tf2::fromMsg(m.transform, freezer.map_odom_);
      freezer.cached_ = true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  if (!freezer.odom_received_) {
    return false;   // no odometry sample yet
  }
  map_base_out = freezer.map_odom_ * freezer.odom_base_;
  return true;
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
  attach_odom_source(node, freezer_);
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

  tf2::Transform map_base;
  if (!compose_map_base(config(), *tf_buffer_, freezer_, map_base)) {
    RCLCPP_WARN(node->get_logger(), "tolerance check: TF unavailable");
    return BT::NodeStatus::FAILURE;
  }

  const DockFrameError e = compute_dock_frame_error(dock, map_base);
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
