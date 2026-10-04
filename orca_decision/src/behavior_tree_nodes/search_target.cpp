#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>
#include <cmath>

namespace {

constexpr double kDegToRad = M_PI / 180.0;

// 單次 tick 的 dt 上限。決策迴圈約 50 Hz，正常 dt 是 0.02 s；超過這個值代表
// 中間有一段時間沒 tick 到本節點（樹在跑別的節點），那段不能算推進時間。
constexpr double kMaxTickDt = 0.2;

// 偏離扇區超過 sweep_half + 這個餘裕才進「回正」。沒有餘裕的話，擺動到邊界
// 反向時的超調會在 advance / recenter 之間來回跳。
constexpr double kRecentreMarginRad = 10.0 * kDegToRad;

// 回正時的轉向 P 增益，輸出夾在 ±max(yaw_speed, kRecentreMinYawSpeed)。
// 不能直接沿用 yaw_speed：找桶用 0.2 慢掃，第一次校正跑實測回正 166° → 94°
// 花了 30 秒（約 2.4°/s）。慢掃是為了偵測，回正只是轉向，不需要一起慢。
constexpr double kRecentreGain = 1.0;
constexpr double kRecentreMinYawSpeed = 0.5;

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

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
        BT::InputPort<int>("stable_frames", 2, "Number of frames target must be centered to return SUCCESS"),
        // 前進掃描。heading_deg 刻意不給預設值：沒寫 = 原本的原地轉圈，
        // 既有呼叫端（資格賽各樹）行為完全不變。
        BT::InputPort<double>("heading_deg",
                              "Sweep centre, degrees relative to mission_start_yaw "
                              "(0 = toward gate at start). Unset = spin in place"),
        BT::InputPort<double>("sweep_half_deg", 60.0, "Half-width of the sweep sector"),
        BT::InputPort<double>("advance_surge", 0.0, "Surge command while sweeping inside the sector"),
        BT::InputPort<double>("advance_budget_sec", 0.0,
                              "Total seconds of forward thrust allowed, accumulated across "
                              "retries; when spent, fall back to spinning in place"),
        BT::InputPort<double>("settle_sec", 0.0,
                              "On each entry, withhold forward thrust for this many seconds "
                              "(sweep and detection continue) so carried momentum dies out")
    };
}

// 沒看到目標時的前進掃描：以 centre_yaw 為中心左右擺 ±sweep_half，在扇區內
// 才給 surge。迴轉超出扇區與回正時都不前進，不會橫著或往後衝向池壁。
//
// 這裡不做避障：用到前進掃描的兩段過門都包在 ReactiveFallback[AvoidObstacle]
// 底下，找桶那段在門後、沒有 flare。撞柱段刻意不用前進掃描（目標常在側後方、
// 橘 flare 反而常在 180° 扇區內，見 trees.xml CommunicationTask 的註解）。
MotionCommand SearchTarget::advanceSweep(double yaw_speed, double centre_yaw, double sweep_half,
                                         float advance_surge, double advance_budget,
                                         double settle_left, double dt) {
    MotionCommand cmd;
    const double yaw = ctx_->world_model->getYaw();
    // 正值 = 目前航向在中心的「yaw 增加」那一側。cmd.yaw 為正會讓 yaw 增加
    // （TurnToYaw / BlindForward 用的是同一個約定）。
    const double off = wrapAngle(yaw - centre_yaw);

    if (std::abs(off) > sweep_half + kRecentreMarginRad) {
        const double limit = std::max(yaw_speed, kRecentreMinYawSpeed);
        cmd.yaw = static_cast<float>(std::clamp(-kRecentreGain * off, -limit, limit));
        // 回正後從靠近的那一側開始往另一側擺，不用再折返。
        sweep_direction_ = (off > 0) ? -1 : 1;
        ctx_->debug_msg += " [recenter off=" + std::to_string(static_cast<int>(off / kDegToRad))
                         + "deg]";
        return cmd;
    }

    if (off > sweep_half) {
        sweep_direction_ = -1;
    } else if (off < -sweep_half) {
        sweep_direction_ = 1;
    }
    cmd.yaw = static_cast<float>(yaw_speed * sweep_direction_);

    // 停穩期間只轉不推，也不扣預算 —— 預算是「主動推進」的上限，慣性滑行
    // 不該吃掉它，否則停穩之後能走的距離就比設計的少。
    if (settle_left > 0.0) {
        ctx_->debug_msg += " [settle " + std::to_string(static_cast<int>(std::ceil(settle_left)))
                         + "s left]";
        return cmd;
    }

    if (std::abs(off) <= sweep_half) {
        cmd.surge = advance_surge;
        advance_used_sec_ += dt;
    }

    ctx_->debug_msg += " [advance used=" + std::to_string(static_cast<int>(advance_used_sec_))
                     + "/" + std::to_string(static_cast<int>(advance_budget)) + "s]";
    return cmd;
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

    // 前進掃描參數。heading_deg 沒寫就整段停用。
    // 「XML 沒寫」與「寫了但解析失敗」要分開：前者是刻意停用，後者是設定錯誤。
    // 不分的話，後者會靜默退回原地轉圈 —— 樹照跑、沒有任何錯誤訊息，只能從
    // debug 少了 [advance] 標記事後察覺（第一次校正跑就是這樣浪費掉的）。
    double heading_deg = 0.0;
    const bool heading_written = config().input_ports.count("heading_deg") > 0;
    const auto heading_res = getInput<double>("heading_deg", heading_deg);
    const bool advance_enabled = static_cast<bool>(heading_res);
    if (heading_written && !advance_enabled) {
        RCLCPP_ERROR_THROTTLE(ctx_->node->get_logger(), *ctx_->node->get_clock(), 5000,
                              "SearchTarget[%s]: heading_deg is set in XML but unresolved, "
                              "advancing sweep DISABLED: %s",
                              name().c_str(), heading_res.error().c_str());
    }
    double sweep_half_deg = 60.0;
    getInput<double>("sweep_half_deg", sweep_half_deg);
    double advance_surge = 0.0;
    getInput<double>("advance_surge", advance_surge);
    double advance_budget = 0.0;
    getInput<double>("advance_budget_sec", advance_budget);
    double settle_sec = 0.0;
    getInput<double>("settle_sec", settle_sec);

    // 任務重新開始時，累計的推進秒數歸零。用 nanoseconds 比較，因為
    // mission_start_time 預設建構的 clock type 跟 now() 不同，直接比會丟例外。
    const int64_t mission_stamp = ctx_->mission_start_time.nanoseconds();
    if (mission_stamp != advance_mission_stamp_ns_) {
        advance_mission_stamp_ns_ = mission_stamp;
        advance_used_sec_ = 0.0;
    }

    const rclcpp::Time now = ctx_->node->now();
    double dt = 0.0;
    if (have_last_tick_) {
        dt = std::clamp((now - last_tick_time_).seconds(), 0.0, kMaxTickDt);
    } else {
        // 新的一次進入（第一次 tick、halt 之後、或上次 SUCCESS 之後）。
        // settle 從這裡起算：每次重新進來，前面多半是 BlindForward 或
        // ApproachTarget 在推，艇身上帶著慣性。
        entry_time_ = now;
    }
    last_tick_time_ = now;
    have_last_tick_ = true;

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
                // SUCCESS 不會呼叫 halt，同樣要清 dt 起點：下次重新進入前
                // ApproachTarget 等節點跑掉的時間不能算進來。
                have_last_tick_ = false;
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

    stable_frames_ = 0;

    // Not found: sweep forward within the sector while budget remains
    if (advance_enabled && advance_used_sec_ < advance_budget) {
        const double centre_yaw =
            wrapAngle(ctx_->mission_start_yaw + heading_deg * kDegToRad);
        const double settle_left = settle_sec - (now - entry_time_).seconds();
        ctx_->wrench_adapter->setCommand(advanceSweep(
            yaw_speed, centre_yaw, sweep_half_deg * kDegToRad,
            static_cast<float>(advance_surge), advance_budget, settle_left, dt));
        return BT::NodeStatus::RUNNING;
    }

    // Not found and no budget left (or advance disabled): yaw sweep in place
    if (advance_enabled) {
        ctx_->debug_msg += " [spin budget spent]";
    }
    MotionCommand cmd;
    cmd.yaw = static_cast<float>(yaw_speed * sweep_direction_);
    ctx_->wrench_adapter->setCommand(cmd);

    return BT::NodeStatus::RUNNING;
}

void SearchTarget::halt() {
    stable_frames_ = 0;
    // 推進累計（advance_used_sec_）刻意保留，見 header。只清 dt 的起點。
    have_last_tick_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
