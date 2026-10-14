#ifndef WR_BRINGUP__CHECKDOCKED_HPP_
#define WR_BRINGUP__CHECKDOCKED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace wr_bringup
{

// Condition node: SUCCESS when the robot's LIVE pose (TF map->base_footprint,
// the same source the goal checker uses - unlike /amcl_pose, TF is not stale
// at standstill) is within `tolerance` (metres) of the `dock` pose.
// Logs the measured positional error and yaw error every tick.
class CheckDocked : public BT::ConditionNode
{
public:
  explicit CheckDocked(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock", "dock pose to verify"),
      BT::InputPort<double>("tolerance", 0.10, "positional tolerance in metres")
    };
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__CHECKDOCKED_HPP_
