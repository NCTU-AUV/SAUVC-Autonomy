#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

FinalAlignTarget::FinalAlignTarget(const std::string &name,
                                   const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList FinalAlignTarget::providedPorts() {
  return {BT::InputPort<std::string>("label")};
}

BT::NodeStatus FinalAlignTarget::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label;
  if (!getInput<std::string>("label", label)) {
    throw BT::RuntimeError("missing required input [label]");
  }

  ctx_->current_action = name();
  ctx_->target_label = label;

  // Single source of truth for the detection image centre — see decision_node's
  // declaration. Do not reintroduce a local constant here.
  const float centre_x =
      static_cast<float>(ctx_->node->get_parameter("image_center_x").as_double());
  const float centre_y =
      static_cast<float>(ctx_->node->get_parameter("image_center_y").as_double());

  auto obj = ctx_->world_model->getObjectNearestImageCenter(label, centre_x, centre_y);
  if (!obj.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Target lost: " + label + " (" + std::to_string(lost_frames_) + "/8)";
    if (lost_frames_ > 8) {
      lost_frames_ = 0;
      aligning_ = false;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::RUNNING;
  }
  
  lost_frames_ = 0;

  ctx_->debug_msg = "Aligning to: " + label;

  // Yaw error from the box centre in the 640-wide tensor. Target right of
  // centre -> error_x < 0 -> positive yaw -> starboard turn (down-positive frame).
  float error_x = centre_x - obj->cx;

  // Check if within threshold
  if (std::abs(error_x) < 20.0f) { // pixel threshold
    if (!aligning_) {
      aligning_ = true;
      align_start_time_ = ctx_->node->now();
    } else {
      auto elapsed = (ctx_->node->now() - align_start_time_).seconds();
      if (elapsed >= 0.15) {
        aligning_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else {
    aligning_ = false;
  }

  // Align logic with deadband and damping to prevent yaw hunting/oscillation
  MotionCommand cmd;
  cmd.surge = 0.0f;
  if (std::abs(error_x) < 12.0f) {
    cmd.yaw = 0.0f; // Inside deadband: zero torque to let hydrodynamic drag stabilize
  } else {
    cmd.yaw = std::clamp(-0.003f * error_x, -0.25f, 0.25f);
  }

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void FinalAlignTarget::halt() {
  aligning_ = false;
  lost_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
