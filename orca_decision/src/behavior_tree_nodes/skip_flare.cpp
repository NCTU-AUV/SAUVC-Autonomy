#include "orca_decision/behavior_tree_nodes.hpp"

SkipFlare::SkipFlare(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList SkipFlare::providedPorts() {
    return {};
}

BT::NodeStatus SkipFlare::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    if (ctx_) {
        ctx_->current_action = name();
        ctx_->debug_msg = "SkipFlare: flare not found, skipping";
        RCLCPP_WARN(ctx_->node->get_logger(),
                    "SkipFlare: Failed to find/align [%s] after retries, skipping current flare",
                    ctx_->target_label.c_str());
    }

    return BT::NodeStatus::SUCCESS;
}
