#ifndef WR_BRINGUP__BACKUP_HPP_
#define WR_BRINGUP__BACKUP_HPP_

#include <string>

#include "nav2_msgs/action/back_up.hpp"

#include "wr_bringup/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace wr_bringup
{

// Fine-adjustment primitive: drive straight backward by `distance` metres at
// `speed` m/s (nav2 behavior_server "backup" action, dead-reckoned — no
// planning). Used to back out of a missed dock before re-approaching.
class Backup : public wr_bringup::BtActionNode<nav2_msgs::action::BackUp>
{
public:
  explicit Backup(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 0.25, "metres to back up"),
      BT::InputPort<double>("speed", 0.1, "reverse speed in m/s")
    };
  }
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__BACKUP_HPP_
