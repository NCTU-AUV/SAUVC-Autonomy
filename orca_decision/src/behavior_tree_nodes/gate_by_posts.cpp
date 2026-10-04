// Gate traversal driven by a pair of posts rather than by a whole-gate box.
//
// Why this exists: the qualification gate's top bar sits ~8 cm below the
// surface, where surface glare and the waterline routinely destroy it in the
// image. When that happens the detector stops producing one wide "gate" box and
// instead produces one tall, narrow box per vertical post — still labelled
// "gate", because the qualification model has no other class. The original
// SearchTarget/ApproachTarget/FinalAlignTarget trio then steers at whichever
// single post won getObjectNearestImageCenter, i.e. straight at a post rather
// than through the opening.
//
// These three nodes take the same three roles but derive the steering point
// from the midpoint of two posts. They are separate node types, registered
// under separate names: the original trio is untouched and the original
// PassGateProcedure keeps working exactly as before.

#include "orca_decision/behavior_tree_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Same surge schedule as ApproachTarget — see the commentary there for why
// kMinSurge is 2.0 (thruster PWM deadband) and why overshoot coasts instead of
// pushing. Kept in sync deliberately: the two nodes drive the same vehicle
// toward the same target, only the way they measure it differs.
constexpr float kBlindSurge = 8.0f;
constexpr float kMaxSurge = 20.0f;
constexpr float kMinSurge = 2.0f;
constexpr int kMaxBlindFrames = 300;  // 30 s at the BT's 10 Hz tick
constexpr int kMaxLostFrames = 8;

GatePostCriteria readCriteria(rclcpp::Node* node) {
  GatePostCriteria criteria;
  criteria.min_aspect_ratio = static_cast<float>(
      node->get_parameter("gate_post_min_aspect_ratio").as_double());
  criteria.min_gap_px = static_cast<float>(
      node->get_parameter("gate_post_min_gap_px").as_double());
  criteria.max_gap_px = static_cast<float>(
      node->get_parameter("gate_post_max_gap_px").as_double());
  criteria.max_height_ratio = static_cast<float>(
      node->get_parameter("gate_post_max_height_ratio").as_double());
  return criteria;
}

float imageCentreX(rclcpp::Node* node) {
  return static_cast<float>(node->get_parameter("image_center_x").as_double());
}

// Split the `label` port on commas. One label is the common case ("gate"); a
// list is what makes flare-based gate detection possible, because the flare
// classes are per-colour and the two posts of one gate routinely come back as
// different colours under uneven lighting.
std::vector<std::string> parseLabels(const std::string& spec) {
  std::vector<std::string> labels;
  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const size_t begin = item.find_first_not_of(" \t");
    if (begin == std::string::npos) continue;  // blank field, e.g. a trailing comma
    const size_t end = item.find_last_not_of(" \t");
    labels.push_back(item.substr(begin, end - begin + 1));
  }
  return labels;
}

}  // namespace

// ============================================================
// SearchGateByPosts
// ============================================================

SearchGateByPosts::SearchGateByPosts(const std::string& name,
                                     const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList SearchGateByPosts::providedPorts() {
  return {
      BT::InputPort<std::string>("label", "gate",
                                 "Comma-separated detection labels the posts may carry, e.g.\n"
              "\"gate\" or \"red_flare,blue_flare,yellow_flare,orange_flare\""),
      BT::InputPort<double>("yaw_speed", 0.4, "Yaw sweeping speed in rad/s"),
      BT::InputPort<double>("center_threshold", 120.0,
                            "Pixel error below which the pair counts as centred"),
      BT::InputPort<int>("stable_frames", 2,
                         "Frames the pair must stay centred before SUCCESS")};
}

BT::NodeStatus SearchGateByPosts::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label = "gate";
  getInput<std::string>("label", label);

  double yaw_speed = 0.4;
  getInput<double>("yaw_speed", yaw_speed);

  double center_threshold = 120.0;
  getInput<double>("center_threshold", center_threshold);

  int required_stable_frames = 2;
  getInput<int>("stable_frames", required_stable_frames);

  ctx_->current_action = name();
  ctx_->target_label = label;

  auto pair = ctx_->world_model->getGatePostPair(parseLabels(label), readCriteria(ctx_->node));

  if (pair.has_value()) {
    const float error_x = imageCentreX(ctx_->node) - pair->center_cx;

    ctx_->debug_msg = "Post pair found: gap=" + std::to_string(pair->gap_px) +
                      "px err=" + std::to_string(error_x);

    if (std::abs(error_x) < center_threshold) {
      stable_frames_++;
      if (stable_frames_ >= required_stable_frames) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      stable_frames_ = 0;
    }

    MotionCommand cmd;
    cmd.yaw = std::clamp(-0.0035f * error_x, -0.35f, 0.35f);
    cmd.surge = 0.0f;
    ctx_->wrench_adapter->setCommand(cmd);

    // Remember which way the opening lay, so a momentary dropout resumes the
    // sweep toward it instead of away from it.
    sweep_direction_ = (error_x < 0) ? 1 : -1;

    return BT::NodeStatus::RUNNING;
  }

  stable_frames_ = 0;
  ctx_->debug_msg = "Searching gate posts: " + label;

  MotionCommand cmd;
  cmd.yaw = static_cast<float>(yaw_speed * sweep_direction_);
  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void SearchGateByPosts::halt() {
  stable_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}

// ============================================================
// ApproachGateByPosts
// ============================================================

ApproachGateByPosts::ApproachGateByPosts(const std::string& name,
                                         const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList ApproachGateByPosts::providedPorts() {
  return {BT::InputPort<std::string>("label", "gate",
                                     "Comma-separated detection labels the posts may carry, e.g.\n"
              "\"gate\" or \"red_flare,blue_flare,yellow_flare,orange_flare\""),
          BT::InputPort<double>("distance",
                                "Stop this many metres short of the gate"),
          BT::InputPort<double>(
              "success_gap_px", -1.0,
              "Post separation that stands in for `distance` when depth never "
              "resolves; <0 takes the gate_posts_success_gap_px parameter")};
}

BT::NodeStatus ApproachGateByPosts::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label = "gate";
  getInput<std::string>("label", label);

  double target_distance;
  if (!getInput<double>("distance", target_distance)) {
    throw BT::RuntimeError("missing required input [distance]");
  }

  double success_gap_px = -1.0;
  getInput<double>("success_gap_px", success_gap_px);
  if (success_gap_px < 0.0) {
    success_gap_px =
        ctx_->node->get_parameter("gate_posts_success_gap_px").as_double();
  }

  ctx_->current_action = name();
  ctx_->target_label = label;

  auto pair = ctx_->world_model->getGatePostPair(parseLabels(label), readCriteria(ctx_->node));
  if (!pair.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Post pair lost (" + std::to_string(lost_frames_) + "/" +
                      std::to_string(kMaxLostFrames) + ")";
    if (lost_frames_ > kMaxLostFrames) {
      lost_frames_ = 0;
      blind_frames_ = 0;
      stable_frames_ = 0;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::RUNNING;
  }

  lost_frames_ = 0;

  const float error_x = imageCentreX(ctx_->node) - pair->center_cx;
  const bool depth_resolved = pair->distance > 0.0f;

  // Closure test. Depth is preferred, but depth_perception runs its gate-
  // specific column estimator on anything labelled "gate", and a single narrow
  // post often fails its column-span check and comes back as -1. The post
  // separation is the fallback: for a fixed-width gate it grows monotonically
  // as the vehicle closes in, needs no intrinsics, and is exactly the quantity
  // these nodes already compute.
  const bool close_enough = depth_resolved
                                ? (pair->distance < target_distance)
                                : (pair->gap_px >= success_gap_px);

  if (close_enough) {
    stable_frames_++;
    if (stable_frames_ >= 5) {
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_frames_ = 0;
  }

  MotionCommand cmd;

  if (std::abs(error_x) < 15.0f) {
    cmd.yaw = 0.0f;
  } else {
    cmd.yaw = std::clamp(-0.0035f * error_x, -0.3f, 0.3f);
  }

  if (depth_resolved) {
    blind_frames_ = 0;
    ctx_->debug_msg = "Approaching gate posts: dist=" +
                      std::to_string(pair->distance) +
                      " gap=" + std::to_string(pair->gap_px);

    const float dist_error = pair->distance - static_cast<float>(target_distance);
    if (dist_error <= 0.0f) {
      cmd.surge = 0.0f;  // overshot: coast rather than push further in
    } else if (std::abs(error_x) > 60.0f) {
      cmd.surge = 0.0f;  // too far off-centre: turn first, don't arc
    } else {
      cmd.surge = std::clamp(8.0f * dist_error, kMinSurge, kMaxSurge);
    }
  } else {
    blind_frames_++;
    ctx_->debug_msg = "Approaching gate posts (visual): gap=" +
                      std::to_string(pair->gap_px) + "/" +
                      std::to_string(success_gap_px) + " (" +
                      std::to_string(blind_frames_) + "/" +
                      std::to_string(kMaxBlindFrames) + ")";
    if (blind_frames_ > kMaxBlindFrames) {
      blind_frames_ = 0;
      stable_frames_ = 0;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    cmd.surge = (std::abs(error_x) > 60.0f) ? 0.0f : kBlindSurge;
  }

  ctx_->wrench_adapter->setCommand(cmd);
  return BT::NodeStatus::RUNNING;
}

void ApproachGateByPosts::halt() {
  // Clear every counter, not just stable_frames_: halt() is what the enclosing
  // Timeout calls on expiry and RetryUntilSuccessful re-enters immediately
  // afterwards, so a carried-over count would make the next attempt give up
  // early on ticks it did not spend.
  stable_frames_ = 0;
  lost_frames_ = 0;
  blind_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}

// ============================================================
// AlignGateByPosts
// ============================================================

AlignGateByPosts::AlignGateByPosts(const std::string& name,
                                   const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList AlignGateByPosts::providedPorts() {
  return {BT::InputPort<std::string>("label", "gate",
                                     "Comma-separated detection labels the posts may carry, e.g.\n"
              "\"gate\" or \"red_flare,blue_flare,yellow_flare,orange_flare\"")};
}

BT::NodeStatus AlignGateByPosts::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label = "gate";
  getInput<std::string>("label", label);

  ctx_->current_action = name();
  ctx_->target_label = label;

  auto pair = ctx_->world_model->getGatePostPair(parseLabels(label), readCriteria(ctx_->node));
  if (!pair.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Post pair lost while aligning (" +
                      std::to_string(lost_frames_) + "/" +
                      std::to_string(kMaxLostFrames) + ")";
    if (lost_frames_ > kMaxLostFrames) {
      lost_frames_ = 0;
      aligning_ = false;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::RUNNING;
  }

  lost_frames_ = 0;

  const float error_x = imageCentreX(ctx_->node) - pair->center_cx;
  ctx_->debug_msg = "Aligning to gate centre: err=" + std::to_string(error_x) +
                    " gap=" + std::to_string(pair->gap_px);

  if (std::abs(error_x) < 20.0f) {
    if (!aligning_) {
      aligning_ = true;
      align_start_time_ = ctx_->node->now();
    } else if ((ctx_->node->now() - align_start_time_).seconds() >= 0.15) {
      aligning_ = false;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    aligning_ = false;
  }

  MotionCommand cmd;
  cmd.surge = 0.0f;
  if (std::abs(error_x) < 12.0f) {
    cmd.yaw = 0.0f;  // deadband: let hydrodynamic drag settle the heading
  } else {
    cmd.yaw = std::clamp(-0.003f * error_x, -0.25f, 0.25f);
  }
  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void AlignGateByPosts::halt() {
  aligning_ = false;
  lost_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
