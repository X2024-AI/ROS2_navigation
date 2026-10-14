#ifndef WR_BRINGUP__MOVETOPOSE_HPP_
#define WR_BRINGUP__MOVETOPOSE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "wr_bringup/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace wr_bringup
{

// Wraps the Nav2 NavigateToPose action. Optionally selects the Nav2-side
// behavior tree per goal via the `bt_xml` port (empty = default tree).
class MoveToPose : public wr_bringup::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit MoveToPose(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("bt_xml", "", "Nav2-side BT to run for this goal")
    };
  }
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__MOVETOPOSE_HPP_
