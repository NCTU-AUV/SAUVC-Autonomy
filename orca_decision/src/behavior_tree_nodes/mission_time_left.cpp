#include "orca_decision/behavior_tree_nodes.hpp"

MissionTimeLeft::MissionTimeLeft(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
}

BT::PortsList MissionTimeLeft::providedPorts() {
    return { BT::InputPort<double>("seconds", 60.0,
                                   "需要的剩餘秒數；剩餘時間 >= 此值才回 SUCCESS") };
}

BT::NodeStatus MissionTimeLeft::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double need = 60.0;
    getInput<double>("seconds", need);

    // 任務還沒開始時 mission_start_time 沒有意義（它在 startMissionCallback
    // 才被設定）。這種情況一律放行 —— 唯一會走到這裡的是單獨測試某棵子樹，
    // 讓時間守衛擋下測試沒有任何好處。
    if (!ctx_ || !ctx_->mission_started) {
        return BT::NodeStatus::SUCCESS;
    }

    double budget = ctx_->node->get_parameter("mission_budget_sec").as_double();
    double elapsed = (ctx_->node->now() - ctx_->mission_start_time).seconds();
    double remaining = budget - elapsed;

    ctx_->debug_msg = "MissionTimeLeft: remain=" + std::to_string(remaining)
                    + " need=" + std::to_string(need);

    if (remaining >= need) {
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(ctx_->node->get_logger(),
                "MissionTimeLeft: %.1f s left, need %.1f s -> FAILURE",
                remaining, need);
    return BT::NodeStatus::FAILURE;
}
