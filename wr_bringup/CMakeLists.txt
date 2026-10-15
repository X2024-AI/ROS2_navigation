cmake_minimum_required(VERSION 3.8)
project(wr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(br2_tiago REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY launch config behavior_tree_xml
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/waypoint_patrol.py scripts/bringup_m6.sh scripts/tuck_arm_retry.sh scripts/odom_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME})

# ---- M6: behavior-tree parking nodes ----
set(BT_DEPENDENCIES
  rclcpp rclcpp_action behaviortree_cpp_v3 action_msgs geometry_msgs
  nav2_msgs ament_index_cpp tf2 tf2_ros tf2_geometry_msgs)

add_library(wr_get_dock_pose_bt_node SHARED src/GetDockPose.cpp)
add_library(wr_move_to_pose_bt_node SHARED src/MoveToPose.cpp)
add_library(wr_check_docked_bt_node SHARED src/CheckDocked.cpp)
add_library(wr_dock_approach_bt_node SHARED src/DockApproach.cpp)
add_library(wr_backup_bt_node SHARED src/Backup.cpp)
add_library(wr_check_pose_tol_bt_node SHARED src/CheckPoseTolerance.cpp)
add_library(wr_calc_pose_error_bt_node SHARED src/CalcPoseError.cpp)
add_library(wr_micro_adjust_bt_node SHARED src/MicroAdjustment.cpp)
add_library(wr_stable_verify_bt_node SHARED src/StableDockVerify.cpp)

set(BT_PLUGINS
  wr_get_dock_pose_bt_node
  wr_move_to_pose_bt_node
  wr_check_docked_bt_node
  wr_dock_approach_bt_node
  wr_backup_bt_node
  wr_check_pose_tol_bt_node
  wr_calc_pose_error_bt_node
  wr_micro_adjust_bt_node
  wr_stable_verify_bt_node)

foreach(bt_plugin ${BT_PLUGINS})
  ament_target_dependencies(${bt_plugin} ${BT_DEPENDENCIES})
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(odom_tf_bridge_cpp src/odom_tf_bridge.cpp)
ament_target_dependencies(odom_tf_bridge_cpp rclcpp geometry_msgs nav_msgs tf2_ros)

add_executable(parking_bt src/parking_bt_main.cpp)
ament_target_dependencies(parking_bt ${BT_DEPENDENCIES})
target_link_libraries(parking_bt zmq)
target_include_directories(parking_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS ${BT_PLUGINS} odom_tf_bridge_cpp parking_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment this line when this package is in a git repo and cpplint should be checked
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
