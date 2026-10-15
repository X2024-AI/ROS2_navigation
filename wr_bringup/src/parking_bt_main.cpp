// WarehouseRunner M6: behavior-tree precise parking.
//
// Application-side BT: cautious approach and precise docking via a dedicated
// Nav2-side BT (slow controller + tight checker), then dock verification.
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // accept parking-slot overrides from the command line, e.g.
  //   ros2 run wr_bringup parking_bt --ros-args \
  //     -p dock_pose:="[1.0, -5.5, 1.57]"
  auto node = rclcpp::Node::make_shared("parking_bt",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("wr_get_dock_pose_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_move_to_pose_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_check_docked_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_dock_approach_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_backup_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_check_pose_tol_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_calc_pose_error_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_micro_adjust_bt_node"));
  factory.registerFromPlugin(loader.getOSName("wr_stable_verify_bt_node"));

  const std::string pkg = ament_index_cpp::get_package_share_directory("wr_bringup");
  const std::string xml_file = pkg + "/behavior_tree_xml/parking.xml";
  const std::string normal_bt = pkg + "/config/bt/nav_bt_normal.xml";
  const std::string dock_bt = pkg + "/config/bt/nav_bt_park_dock.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("normal_bt", normal_bt);
  blackboard->set("dock_bt", dock_bt);

  // One tree-wide TF buffer, alive for the whole demo: it caches map->odom
  // during the approach, so the standstill micro-adjustment checks never
  // starve (AMCL only republishes that transform after ~0.25 m of motion).
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  blackboard->set("tf_buffer", tf_buffer);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  // Live monitoring in Groot (Monitor mode, connect to localhost:2666)
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  RCLCPP_INFO(node->get_logger(), "parking BT started (%s)", xml_file.c_str());
  const auto t0 = std::chrono::steady_clock::now();

  rclcpp::Rate rate(10);
  bool ok = false;
  while (rclcpp::ok()) {
    const BT::NodeStatus st = tree.rootNode()->executeTick();
    rclcpp::spin_some(node);
    if (st == BT::NodeStatus::SUCCESS) {ok = true; break;}
    if (st == BT::NodeStatus::FAILURE) {break;}
    rate.sleep();
  }

  const auto dt = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    node->get_logger(), "parking BT finished: %s after %ld s",
    ok ? "SUCCESS" : "FAILURE", dt);

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
