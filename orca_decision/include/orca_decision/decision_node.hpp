#pragma once

#include <geometry_msgs/msg/wrench.hpp>
#include <orca_interface/msg/decision_status.hpp>
#include <orca_interface/msg/perception_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "orca_decision/decision_context.hpp"
#include "orca_decision/world_model.hpp"
#include "orca_decision/wrench_adapter.hpp"

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode();

private:
  void loadParameters();
  void setupInterfaces();
  void registerBehaviorTree();
  void btTickLoop();
  void publishWrenchLoop();
  orca_interface::msg::DecisionStatus buildStatus();
  void publishStatusJson();

  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void
  perceptionCallback(const orca_interface::msg::PerceptionArray::SharedPtr msg);
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void flareOrderCallback(const std_msgs::msg::String::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult
  onPoolDepthParametersSet(const std::vector<rclcpp::Parameter> &params);

  // Context & Logic
  std::shared_ptr<DecisionContext> ctx_;

  // BT
  BT::BehaviorTreeFactory bt_factory_;
  std::unique_ptr<BT::Tree> tree_;

  // ROS Timers
  rclcpp::TimerBase::SharedPtr bt_timer_;
  rclcpp::TimerBase::SharedPtr wrench_timer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<orca_interface::msg::PerceptionArray>::SharedPtr
      perception_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flare_order_sub_;
  // Held for the lifetime of the node: an unheld handle gets its callback
  // deregistered when it goes out of scope, silently turning the GUI's
  // pool-depth updates into no-ops.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      pool_depth_param_cb_handle_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_depth_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hand_pub_;
  rclcpp::Publisher<orca_interface::msg::DecisionStatus>::SharedPtr status_pub_;
  // Same content as status_pub_, serialised as JSON in a std_msgs/String.
  // DecisionStatus lives in orca_interface, which is a package of this repo and
  // is not built into the control container, so the Web GUI node there cannot
  // deserialise it and mission state was the one thing the operator could not
  // see from the GUI. A String crosses that boundary with no shared package.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_json_pub_;

  std::string tree_xml_file_;
  std::string main_tree_id_;
  bool mission_complete_ = false;
  // The JSON mirror is for a human watching a web page, not for control, so it
  // is decimated off the 50 Hz wrench loop rather than given its own timer.
  int status_json_decimation_ = 10;
  int status_json_counter_ = 0;
};
