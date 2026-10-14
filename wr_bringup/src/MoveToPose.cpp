#include <string>

#include "wr_bringup/MoveToPose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace wr_bringup
{

MoveToPose::MoveToPose(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: wr_bringup::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
MoveToPose::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  goal_.pose = goal;

  // Per-goal Nav2-side behavior tree (e.g. the precise-docking tree).
  std::string bt_xml;
  getInput("bt_xml", bt_xml);
  goal_.behavior_tree = bt_xml;
}

BT::NodeStatus
MoveToPose::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation succeeded");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<wr_bringup::MoveToPose>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<wr_bringup::MoveToPose>("MoveToPose", builder);
}
