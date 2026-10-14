#include <cmath>
#include <string>
#include <vector>

#include "wr_bringup/GetParkingPoses.hpp"

#include "rclcpp/rclcpp.hpp"

namespace wr_bringup
{

static geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

GetParkingPoses::GetParkingPoses(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  // Parking slot picked in RViz (Publish Point survey, 2026-10-14).
  // Both headings = staging->dock direction: atan2(-0.529, 0.835) = -0.565 rad.
  std::vector<double> staging{1.3855, -0.1957, -0.5650};
  std::vector<double> dock{2.2200, -0.7248, -0.5650};
  node->get_parameter_or("staging_pose", staging, staging);
  node->get_parameter_or("dock_pose", dock, dock);

  staging_pose_ = make_pose(staging[0], staging[1], staging[2]);
  dock_pose_ = make_pose(dock[0], dock[1], dock[2]);
}

BT::NodeStatus
GetParkingPoses::tick()
{
  setOutput("staging_pose", staging_pose_);
  setOutput("dock_pose", dock_pose_);
  RCLCPP_INFO(
    config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
    "parking slot: staging (%.2f, %.2f), dock (%.2f, %.2f)",
    staging_pose_.pose.position.x, staging_pose_.pose.position.y,
    dock_pose_.pose.position.x, dock_pose_.pose.position.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wr_bringup::GetParkingPoses>("GetParkingPoses");
}
