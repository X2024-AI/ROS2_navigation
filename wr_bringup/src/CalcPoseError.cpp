#include <cmath>
#include <string>

#include "wr_bringup/CalcPoseError.hpp"
#include "wr_bringup/CheckPoseTolerance.hpp"

namespace wr_bringup
{

CalcPoseError::CalcPoseError(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
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
CalcPoseError::tick()
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
    RCLCPP_WARN(node->get_logger(), "error calc: TF unavailable");
    return BT::NodeStatus::FAILURE;
  }
  const DockFrameError e = compute_dock_frame_error(dock, map_base);

  // Priority: largest error RELATIVE to its tolerance first (a 47 cm
  // longitudinal offset outranks a 3 cm lateral one; ordering self-selects
  // as the loop converges).
  const double n_lat = std::fabs(e.lateral) / pos_tol;
  const double n_lon = std::fabs(e.longitudinal) / pos_tol;
  const double n_yaw = std::fabs(e.yaw) / yaw_tol;

  std::string type = "none";
  double magnitude = 0.0;
  if (n_lat >= n_lon && n_lat >= n_yaw && n_lat > 1.0) {
    type = "lateral";
    magnitude = e.lateral;             // + : robot left of axis -> move right
  } else if (n_lon >= n_lat && n_lon >= n_yaw && n_lon > 1.0) {
    type = "longitudinal";
    magnitude = e.longitudinal;        // + : robot ahead -> move back
  } else if (n_yaw > 1.0) {
    type = "yaw";
    magnitude = e.yaw;                 // + : rotated CCW -> spin CW
  }

  RCLCPP_INFO(
    node->get_logger(),
    "error calc: lon %+.4f, lat %+.4f, yaw %+.4f -> %s (%+.4f)",
    e.longitudinal, e.lateral, e.yaw, type.c_str(), magnitude);

  setOutput("correction", type);
  setOutput("magnitude", magnitude);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::CalcPoseError>("CalcPoseError");
}
