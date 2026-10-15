#include <cmath>
#include <string>
#include <vector>

#include "wr_bringup/GetDockPose.hpp"

#include "rclcpp/rclcpp.hpp"

namespace wr_bringup
{

GetDockPose::GetDockPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  std::vector<double> dock{2.2200, -0.7248, -0.5650};   // RViz-surveyed dock slot
  node->get_parameter_or("dock_pose", dock, dock);

  dock_pose_.header.frame_id = "map";
  dock_pose_.pose.position.x = dock[0];
  dock_pose_.pose.position.y = dock[1];
  dock_pose_.pose.orientation.z = std::sin(dock[2] / 2.0);
  dock_pose_.pose.orientation.w = std::cos(dock[2] / 2.0);
}

BT::NodeStatus
GetDockPose::tick()
{
  setOutput("dock_pose", dock_pose_);
  RCLCPP_INFO(
    config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
    "dock pose: (%.2f, %.2f)",
    dock_pose_.pose.position.x, dock_pose_.pose.position.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::GetDockPose>("GetDockPose");
}
