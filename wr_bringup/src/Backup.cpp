#include <string>

#include "wr_bringup/Backup.hpp"

namespace wr_bringup
{

Backup::Backup(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: wr_bringup::BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
}

void
Backup::on_tick()
{
  double distance = 0.25, speed = 0.1;
  getInput("distance", distance);
  getInput("speed", speed);

  goal_.target.x = -distance;   // negative x = reverse
  goal_.speed = static_cast<float>(speed);
  goal_.time_allowance.sec = 10;
}

BT::NodeStatus
Backup::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "backed up, ready to re-approach");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace wr_bringup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<wr_bringup::Backup>(
        name, "backup", config);
    };

  factory.registerBuilder<wr_bringup::Backup>("Backup", builder);
}
