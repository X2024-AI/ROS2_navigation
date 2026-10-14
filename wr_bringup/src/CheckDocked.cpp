#include <cmath>
#include <string>

#include "wr_bringup/CheckDocked.hpp"

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wr_bringup
{

CheckDocked::CheckDocked(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
}

BT::NodeStatus
CheckDocked::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped dock;
  getInput("dock", dock);
  double tolerance = 0.10;
  getInput("tolerance", tolerance);

  // Live pose: same map->odom (AMCL) + odom->base chain the goal checker
  // evaluates. /amcl_pose would be stale at standstill (AMCL only republishes
  // after update_min_d of motion), which once faked a 0.24 m "miss".
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(node->get_logger(), "docking check: TF unavailable (%s)", e.what());
    return BT::NodeStatus::FAILURE;
  }

  const double dx = t.transform.translation.x - dock.pose.position.x;
  const double dy = t.transform.translation.y - dock.pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double yaw = tf2::getYaw(t.transform.rotation);
  const double dock_yaw = tf2::getYaw(dock.pose.orientation);
  const double yaw_err = std::atan2(
    std::sin(yaw - dock_yaw), std::cos(yaw - dock_yaw));

  RCLCPP_INFO(
    node->get_logger(),
    "docking check: position error %.3f m (tol %.3f), yaw error %.3f rad",
    dist, tolerance, yaw_err);

  return dist <= tolerance ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::CheckDocked>("CheckDocked");
}
