#include <cmath>
#include <string>

#include "wr_bringup/StableDockVerify.hpp"
#include "wr_bringup/CheckPoseTolerance.hpp"

namespace wr_bringup
{

StableDockVerify::StableDockVerify(
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
  freezer_ = get_shared_freezer(config(), node);
}

BT::NodeStatus
StableDockVerify::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped dock;
  getInput("dock", dock);
  double pos_tol = 0.02, yaw_tol = 0.00873;
  int hold_frames = 5;
  getInput("pos_tol", pos_tol);
  getInput("yaw_tol", yaw_tol);
  getInput("hold_frames", hold_frames);

  tf2::Transform map_base;
  if (!compose_map_base(config(), *tf_buffer_, *freezer_, map_base)) {
    good_frames_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  const DockFrameError e = compute_dock_frame_error(dock, map_base);

  const double pos_err = std::hypot(e.longitudinal, e.lateral);
  if (pos_err <= pos_tol && std::fabs(e.yaw) <= yaw_tol) {
    good_frames_++;
  } else {
    if (good_frames_ > 0) {
      RCLCPP_INFO(
        node->get_logger(), "stability broken at frame %d/%d (pos %.4f, yaw %+.4f)",
        good_frames_, hold_frames, pos_err, e.yaw);
    }
    good_frames_ = 0;
  }

  if (good_frames_ >= hold_frames) {
    RCLCPP_INFO(
      node->get_logger(),
      "STABLE DOCK: %d consecutive frames within tol (pos %.4f m, yaw %+.4f rad)",
      good_frames_, pos_err, e.yaw);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::StableDockVerify>("StableDockVerify");
}
