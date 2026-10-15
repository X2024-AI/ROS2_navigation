#ifndef WR_BRINGUP__CALCPOSEERROR_HPP_
#define WR_BRINGUP__CALCPOSEERROR_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wr_bringup/CheckPoseTolerance.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace wr_bringup
{

// SyncAction: computes the live dock-frame error and classifies the dominant
// deviation for MicroAdjustment. Priority: lateral > longitudinal > yaw
// (lateral first because its correction temporarily rotates the base;
// yaw is fixed last). Outputs the signed magnitude of the chosen correction.
class CalcPoseError : public BT::SyncActionNode
{
public:
  explicit CalcPoseError(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock", "dock pose"),
      BT::InputPort<double>("pos_tol", 0.02, "position tolerance in metres"),
      BT::InputPort<double>("yaw_tol", 0.00873, "yaw tolerance in radians"),
      BT::OutputPort<std::string>("correction", "lateral|longitudinal|yaw|none"),
      BT::OutputPort<double>("magnitude", "signed error of the chosen term")
    };
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  MapOdomFreezer freezer_;
};

}  // namespace wr_bringup

#endif  // WR_BRINGUP__CALCPOSEERROR_HPP_
