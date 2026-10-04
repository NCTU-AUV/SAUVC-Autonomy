#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <set>

AvoidObstacle::AvoidObstacle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList AvoidObstacle::providedPorts() {
    return {
        BT::InputPort<double>("sticky_time"),
        // 深度失效時的備援「近了」判準：框高（640 張量像素）達到這個值就當成
        // 進入危險距離。0 = 停用，只認深度。
        BT::InputPort<double>("min_height_px", 200.0,
                              "Bbox height (px) treated as danger range when "
                              "depth is unavailable; 0 disables")
    };
}

BT::NodeStatus AvoidObstacle::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double sticky_time = 1.0;
    getInput<double>("sticky_time", sticky_time);

    double min_height_px = 200.0;
    getInput<double>("min_height_px", min_height_px);

    ctx_->current_action = name();
    ctx_->is_recovering = is_sticky_;

    auto objs = ctx_->world_model->getObjects();

    // Single source of truth for the detection image centre — see decision_node's
    // declaration. Do not reintroduce a bare literal here.
    const double centre_x = ctx_->node->get_parameter("image_center_x").as_double();

    std::set<std::string> flares = {"orange_flare", "red_flare", "blue_flare", "yellow_flare"};

    bool danger = false;
    TrackedObject danger_obj;

    for (const auto& obj : objs) {
        if (!flares.count(obj.label)) {
            continue;
        }

        // 「近到會撞」的判斷有兩條路。
        //
        // 深度那條是原本就有的，但它有一個致命的沉默失效：條件含
        // obj.distance > 0，而 distance 估不出來時 depth_perception_node 回的是
        // −1。flare 桿子只有 1.6 cm 寬，細長框正是立體深度最估不出來的形狀 ——
        // 也就是說最需要避障的那個目標，剛好是這個條件永遠不成立的目標，避障
        // 靜默地整個不作用。橘 flare 碰一下就是 attempt 立即終止，不是扣分。
        //
        // 備援那條用框高當距離的代理量，跟 ApproachTarget 的 min_height_px 同一
        // 套路。門檻 200 px 對應約 2.0 m（不拉伸）或 2.7 m（拉伸），跟深度那條
        // 的 2.0 m 對齊或更早觸發 —— 避障早觸發只是多繞一點，晚觸發才會出事。
        const bool close_by_depth =
            obj.distance > 0 && obj.distance < 2.0;
        const bool close_by_size =
            obj.distance <= 0 && min_height_px > 0.0 &&
            static_cast<double>(obj.height) >= min_height_px;

        // 而且要大致在正前方，不是在側邊。200 px 之於 640 寬的張量約是中間 62%。
        if ((close_by_depth || close_by_size) &&
            std::abs(obj.cx - centre_x) < 200.0) {
            danger = true;
            danger_obj = obj;
            break;
        }
    }

    rclcpp::Time now = ctx_->node->now();

    if (danger) {
        is_sticky_ = true;
        last_avoid_time_ = now;
        ctx_->is_recovering = true;
        ctx_->debug_msg = "Avoiding " + danger_obj.label;
        
        MotionCommand cmd;
        cmd.surge = 5.0f; // small forward
        // Sway away from the object: if it is right of centre, sway left
        // (negative); if left of centre, sway right (positive).
        if (danger_obj.cx > centre_x) {
            cmd.sway = -10.0f; 
        } else {
            cmd.sway = 10.0f;
        }
        ctx_->wrench_adapter->setCommand(cmd);
        return BT::NodeStatus::RUNNING;
    }

    if (is_sticky_) {
        auto elapsed = (now - last_avoid_time_).seconds();
        if (elapsed < sticky_time) {
            // Keep running same command
            return BT::NodeStatus::RUNNING;
        } else {
            is_sticky_ = false;
        }
    }

    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::FAILURE; // No danger
}

void AvoidObstacle::halt() {
    is_sticky_ = false;
    if (ctx_) {
        ctx_->is_recovering = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
