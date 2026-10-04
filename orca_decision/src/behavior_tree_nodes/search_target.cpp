#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

SearchTarget::SearchTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList SearchTarget::providedPorts() {
    return { 
        BT::InputPort<std::string>("label"),
        BT::InputPort<double>("yaw_speed", 0.5, "Yaw sweeping speed in rad/s (default: 0.5)"),
        BT::InputPort<int>("sweep_direction", 1,
                           "Initial sweep rotation: 1 = clockwise, -1 = counter-clockwise"),
        BT::InputPort<double>("center_threshold", 120.0, "Error threshold in pixels to consider target centered"),
        BT::InputPort<int>("stable_frames", 2, "Number of frames target must be centered to return SUCCESS")
    };
}

BT::NodeStatus SearchTarget::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string label;
    if (!getInput<std::string>("label", label)) {
        throw BT::RuntimeError("missing required input [label]");
    }

    // 空字串是「有這個埠、但值是空的」，跟埠不存在不同，所以上面那個 throw 抓
    // 不到。實際來源是 WaitForFlareOrder：收到的順序字串少於三個字元時，剩下的
    // flare_2 / flare_3 會被設成 ""（wait_for_flare_order.cpp）。
    //
    // 空 label 永遠匹配不到任何偵測，而本節點找不到目標時是回 RUNNING 不是
    // FAILURE，所以會一路空轉到外層 Timeout 到期 —— 每個空位白燒
    // 3 × 60 s = 180 秒，還是在轉圈耗電。直接回 FAILURE 讓 RetryUntilSuccessful
    // 立刻收斂到 SkipFlare，把時間留給真的有順序的那幾根。
    if (label.empty()) {
        ctx_->current_action = name();
        ctx_->debug_msg = "SearchTarget: empty label, nothing to search";
        RCLCPP_WARN_ONCE(ctx_->node->get_logger(),
                         "SearchTarget: empty label (flare order shorter than 3?), failing fast");
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::FAILURE;
    }

    double yaw_speed = 0.5;
    getInput<double>("yaw_speed", yaw_speed);

    // 掃描起始方向只在這個節點實例的第一次 tick 決定一次。之後 sweep_direction_
    // 由「上次在畫面哪一側看到目標」接手（見下方），若每 tick 都覆寫回埠值，
    // 跟丟之後就會往錯的方向轉，那個回頭找的行為會壞掉。
    if (!sweep_direction_initialised_) {
        int configured_direction = 1;
        getInput<int>("sweep_direction", configured_direction);
        sweep_direction_ = (configured_direction < 0) ? -1 : 1;
        sweep_direction_initialised_ = true;
    }

    double center_threshold = 120.0;
    getInput<double>("center_threshold", center_threshold);

    int required_stable_frames = 2;
    getInput<int>("stable_frames", required_stable_frames);

    ctx_->current_action = name();
    ctx_->target_label = label;
    ctx_->debug_msg = "Searching target: " + label;

    auto obj = ctx_->world_model->getBestObject(label);
    if (obj.has_value()) {
        const float centre_x =
            static_cast<float>(ctx_->node->get_parameter("image_center_x").as_double());
        
        float error_x = centre_x - obj->cx;
        
        if (std::abs(error_x) < center_threshold) {
            stable_frames_++;
            if (stable_frames_ >= required_stable_frames) {
                ctx_->wrench_adapter->setCommand(MotionCommand()); // stop
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            stable_frames_ = 0;
        }

        // Target found but not centered, apply yaw P-control
        MotionCommand cmd;
        cmd.yaw = std::clamp(-0.0035f * error_x, -0.35f, 0.35f);
        cmd.surge = 0.0f; // Ensure surge is zero while aligning
        ctx_->wrench_adapter->setCommand(cmd);

        // Update sweep direction based on where we last saw it
        // If error_x < 0 (target is on the right), we need to yaw positive (starboard)
        sweep_direction_ = (error_x < 0) ? 1 : -1;

        return BT::NodeStatus::RUNNING;
    }

    // Not found, perform yaw sweep
    stable_frames_ = 0;
    MotionCommand cmd;
    cmd.yaw = static_cast<float>(yaw_speed * sweep_direction_);
    ctx_->wrench_adapter->setCommand(cmd);

    return BT::NodeStatus::RUNNING;
}

void SearchTarget::halt() {
    stable_frames_ = 0;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
