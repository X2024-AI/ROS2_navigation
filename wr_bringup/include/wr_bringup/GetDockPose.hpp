#ifndef WR_BRINGUP__GETDOCKPOSE_HPP_
#define WR_BRINGUP__GETDOCKPOSE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace wr_bringup
{

// Publishes the parking dock pose to the blackboard. The pose comes from the
// node parameter `dock_pose` as [x, y, yaw_rad]. (The staging pose is gone:
// the slow approach is now a dedicated cautious controller plugin selected
// by the Nav2-side docking BT.)
class GetDockPose : public BT::SyncActionNode
{
public:
  explicit GetDockPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose")
    };
  }

private:
  geometry_msgs::msg::PoseStamped dock_pose_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__GETDOCKPOSE_HPP_
