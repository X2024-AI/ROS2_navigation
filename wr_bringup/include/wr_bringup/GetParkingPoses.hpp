#ifndef WR_BRINGUP__GETPARKINGPOSES_HPP_
#define WR_BRINGUP__GETPARKINGPOSES_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace wr_bringup
{

// Publishes the parking slot's staging (coarse approach) pose and the final
// dock pose to the blackboard. Poses come from the node parameters
// `staging_pose` / `dock_pose` as [x, y, yaw_rad] (defaults: home dock).
class GetParkingPoses : public BT::SyncActionNode
{
public:
  explicit GetParkingPoses(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("staging_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose")
    };
  }

private:
  geometry_msgs::msg::PoseStamped staging_pose_;
  geometry_msgs::msg::PoseStamped dock_pose_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__GETPARKINGPOSES_HPP_
