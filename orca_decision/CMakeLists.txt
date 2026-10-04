cmake_minimum_required(VERSION 3.8)
project(orca_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orca_interface REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)

# Eigen3 is header-only but good to find it
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/main.cpp
  src/decision_node.cpp
  src/world_model.cpp
  src/wrench_adapter.cpp
  src/behavior_tree_nodes/search_target.cpp
  src/behavior_tree_nodes/approach_target.cpp
  src/behavior_tree_nodes/final_align_target.cpp
  src/behavior_tree_nodes/blind_forward.cpp
  src/behavior_tree_nodes/turn_to_yaw.cpp
  src/behavior_tree_nodes/avoid_obstacle.cpp
  src/behavior_tree_nodes/set_camera.cpp
  src/behavior_tree_nodes/set_depth.cpp
  src/behavior_tree_nodes/move_above_target.cpp
  src/behavior_tree_nodes/check_bottom_clear.cpp
  src/behavior_tree_nodes/drop_ball.cpp
  src/behavior_tree_nodes/wait_for_flare_order.cpp
  src/behavior_tree_nodes/bump_flare.cpp
  src/behavior_tree_nodes/skip_flare.cpp
  src/behavior_tree_nodes/go_to_pose.cpp
  src/behavior_tree_nodes/search_bottom_target.cpp
  src/behavior_tree_nodes/spiral_search_bottom.cpp
  src/behavior_tree_nodes/extend_arm.cpp
  src/behavior_tree_nodes/grab_ball.cpp
  src/behavior_tree_nodes/retract_arm.cpp
  src/behavior_tree_nodes/wait_for_start.cpp
  src/behavior_tree_nodes/finish_mission.cpp
  src/behavior_tree_nodes/gate_by_posts.cpp
  src/behavior_tree_nodes/mission_time_left.cpp
)

add_executable(decision_node ${SOURCES})
target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(decision_node
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  orca_interface
  behaviortree_cpp_v3
  tf2
)

install(TARGETS decision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
