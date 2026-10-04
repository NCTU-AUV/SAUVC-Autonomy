#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <algorithm>

static inline double normalizeAngleBump(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

BumpFlare::BumpFlare(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList BumpFlare::providedPorts() {
    return { BT::InputPort<double>("timeout") };
}

BT::NodeStatus BumpFlare::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    // 優先序：XML 的 timeout port > bump_flare_timeout_sec 參數。
    //
    // 原本是反過來的（參數無條件覆蓋 port），而參數永遠 > 0，所以 trees.xml 裡
    // 那三個 timeout 是純裝飾 —— 樹上看得到、調不動，三根柱子也沒辦法各給不同
    // 的撞擊時長。
    //
    // 這裡靠的是 providedPorts() 把 "timeout" 宣告成「沒有預設值」：XML 沒寫這
    // 個屬性時 getInput 回 false，才落回參數。所以既有那些沒帶 timeout 的呼叫端
    // 行為不變，仍然吃 YAML。
    double timeout = 0.0;
    if (!getInput<double>("timeout", timeout) || timeout <= 0.0) {
        timeout = ctx_->node->get_parameter("bump_flare_timeout_sec").as_double();
    }
    if (timeout <= 0.0) timeout = 8.0;

    double bump_surge = ctx_->node->get_parameter("bump_flare_surge").as_double();

    ctx_->current_action = name();
    ctx_->debug_msg = "Bumping flare";

    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
        target_yaw_ = ctx_->world_model->getYaw();
        RCLCPP_INFO(ctx_->node->get_logger(),
                    "BumpFlare: Starting to bump [%s] (timeout=%.1f s, surge=%.2f)",
                    ctx_->target_label.c_str(), timeout, bump_surge);
    }

    double elapsed = (now - start_time_).seconds();
    if (elapsed >= timeout) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "BumpFlare: completed";
        RCLCPP_INFO(ctx_->node->get_logger(),
                    "BumpFlare: Successfully bumped [%s]! (duration=%.1f s)",
                    ctx_->target_label.c_str(), elapsed);
        return BT::NodeStatus::SUCCESS;
    }

    // Surge only with heading lock
    MotionCommand cmd;
    cmd.surge = static_cast<float>(bump_surge);

    // Heading lock
    double current_yaw = ctx_->world_model->getYaw();
    double yaw_error = normalizeAngleBump(target_yaw_ - current_yaw);
    cmd.yaw = std::clamp(static_cast<float>(1.0 * yaw_error), -0.5f, 0.5f);

    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "BumpFlare: t=" + std::to_string(elapsed) + "/" + std::to_string(timeout);

    return BT::NodeStatus::RUNNING;
}

void BumpFlare::halt() {
    started_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
