#include "orca_decision/behavior_tree_nodes.hpp"

#include <algorithm>

namespace {
// 破水面保險。艇體上緣在原點上方 0.23 m（orca_auv/model.sdf：主體圓柱中心
// +0.115、半徑 0.115），而深度回授量的是原點，所以任何小於 0.23 的指令都會
// 讓艇體露出水面 —— 規則上「任何時候破水面 = attempt 立即結束」。0.35 是
// 0.23 再加約 0.12 m 的餘裕，吸收 PID 的過衝和壓力計雜訊。
//
// depth <= 0 不受限：那是 FinishMission 和收尾用的「主動上浮」，是刻意要
// 浮出去的，不能被這條擋掉。
constexpr float kMinDepth = 0.35f;
}  // namespace

SetDepth::SetDepth(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetDepth::providedPorts() {
    // depth 與 zone 擇一：depth 是舊路徑，資格賽三棵樹全部走這條，程式碼原封
    // 不動；zone（"gate"/"drop"/"flare"）是新路徑，只有決賽樹在用，深度由現場
    // 池深換算（ZoneTargetDepth，decision_context.hpp/decision_node.cpp）。
    return { BT::InputPort<float>("depth"), BT::InputPort<std::string>("zone") };
}

BT::NodeStatus SetDepth::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    float depth;
    const bool has_depth = getInput<float>("depth", depth).has_value();
    std::string zone;
    const bool has_zone = getInput<std::string>("zone", zone).has_value();

    if (has_depth == has_zone) {
        // 兩者都缺或兩者都給，都是樹寫錯 —— 寧可載入期就炸掉，不要在比賽現場
        // 才發現深度指令是哪一個沒送出去。
        throw BT::RuntimeError(
            "SetDepth requires exactly one of [depth] or [zone]");
    }

    if (has_zone) {
        if (!ctx_ || !ZoneTargetDepth(*ctx_, zone, &depth)) {
            throw BT::RuntimeError("SetDepth: unknown zone [" + zone + "]");
        }
    }

    const float commanded = (depth > 0.0f) ? std::max(depth, kMinDepth) : depth;

    if (ctx_) {
        // current_depth_zone 記住「這次深度是不是由某個 zone 換算出來的」，
        // 池深參數變更的回呼靠它判斷要不要重發 desired_depth。depth= 字面值
        // 路徑（資格賽、上浮用的 depth="0.0"）清空它，確保那條回呼永遠不會去
        // 動資格賽或收尾階段已經下達的深度指令。
        ctx_->current_depth_zone = has_zone ? zone : std::string();
        ctx_->current_action = name();
        ctx_->debug_msg = "Set depth: " + std::to_string(commanded);
        if (commanded != depth) {
            RCLCPP_WARN(ctx_->node->get_logger(),
                        "SetDepth %.2f m would surface the hull; clamped to %.2f m",
                        depth, commanded);
        }
    }

    if (ctx_ && ctx_->desired_depth_pub) {
        std_msgs::msg::Float64 msg;
        msg.data = commanded;
        ctx_->desired_depth_pub->publish(msg);
    }

    return BT::NodeStatus::SUCCESS;
}
