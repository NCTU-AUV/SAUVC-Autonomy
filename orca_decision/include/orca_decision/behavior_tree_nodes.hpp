#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Dense>
#include "orca_decision/decision_context.hpp"

// Forward declaration of registration function
void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<DecisionContext> ctx);

// ============================================================
// Existing BT Nodes (unchanged)
// ============================================================

class SearchTarget : public BT::ActionNodeBase {
public:
    SearchTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    // 掃描旋轉方向：+1 順時鐘、-1 逆時鐘。第一次 tick 由 sweep_direction 埠
    // 初始化（見 search_target.cpp），之後每看到一次目標就依目標在畫面左右
    // 更新，所以「跟丟後往剛才看到的方向回轉」的行為不受影響。
    int sweep_direction_ = 1;
    bool sweep_direction_initialised_ = false;
};

class ApproachTarget : public BT::ActionNodeBase {
public:
    ApproachTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    int lost_frames_ = 0;
    int blind_frames_ = 0;
    // 深度失效時改用「框有多高」判斷到位（min_height_px 埠）。與 stable_frames_
    // 分開計數是刻意的：兩條到位路徑各自累計，深度時有時無地跳動時才不會互相
    // 把對方的計數清掉、湊不滿任何一邊的門檻。
    int height_stable_frames_ = 0;
};

class FinalAlignTarget : public BT::ActionNodeBase {
public:
    FinalAlignTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time align_start_time_;
    bool aligning_ = false;
    int lost_frames_ = 0;
};

class BlindForward : public BT::ActionNodeBase {
public:
    BlindForward(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    double target_yaw_ = 0.0;
    // 被 halt 打斷前已經走掉的時間。沒有這個累計，避障每觸發一次盲走就從 0 重
    // 新計時，一趟 10 秒的盲走會走成兩趟。
    double elapsed_before_halt_ = 0.0;
    // 航向鎖只在整段盲走的最開頭取一次。避障結束後重新取的話，鎖住的會是閃避
    // 完的歪掉航向，等於把避障的側移永久烙進航向裡。
    bool yaw_locked_ = false;
};

class TurnToYaw : public BT::ActionNodeBase {
public:
    TurnToYaw(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    bool started_ = false;
    double target_yaw_ = 0.0;
};

class AvoidObstacle : public BT::ActionNodeBase {
public:
    AvoidObstacle(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time last_avoid_time_;
    bool is_sticky_ = false;
};

class SetCamera : public BT::SyncActionNode {
public:
    SetCamera(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class SetDepth : public BT::SyncActionNode {
public:
    SetDepth(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

// ============================================================
// New BT Nodes — Task 2: Target Acquisition
// ============================================================

class MoveAboveTarget : public BT::ActionNodeBase {
public:
    MoveAboveTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    rclcpp::Time start_time_;
    bool started_ = false;
    bool target_acquired_ = false;
    float filtered_error_x_ = 0.0f;
    float filtered_error_y_ = 0.0f;
};

class CheckBottomClear : public BT::ActionNodeBase {
public:
    CheckBottomClear(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    int clear_frames_ = 0;
};

class DropBall : public BT::ActionNodeBase {
public:
    DropBall(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time phase_start_;
    bool started_ = false;
    int phase_ = 0;  // 0=stop, 1=release, 2=wait+record
};

// ============================================================
// New BT Nodes — Task 4: Communication & Localization
// ============================================================

class WaitForFlareOrder : public BT::ActionNodeBase {
public:
    WaitForFlareOrder(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time wait_start_time_;
    bool waiting_started_ = false;
};

class BumpFlare : public BT::ActionNodeBase {
public:
    BumpFlare(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    double target_yaw_ = 0.0;
};

class SkipFlare : public BT::SyncActionNode {
public:
    SkipFlare(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

// ============================================================
// New BT Nodes — Task 3: Target Reacquisition
// ============================================================

class GoToPose : public BT::ActionNodeBase {
public:
    GoToPose(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class SearchBottomTarget : public BT::ActionNodeBase {
public:
    SearchBottomTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class SpiralSearchBottom : public BT::ActionNodeBase {
public:
    SpiralSearchBottom(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    double target_yaw_ = 0.0;
};

class ExtendArm : public BT::ActionNodeBase {
public:
    ExtendArm(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class GrabBall : public BT::ActionNodeBase {
public:
    GrabBall(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class RetractArm : public BT::ActionNodeBase {
public:
    RetractArm(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

// ============================================================
// New BT Nodes — Qualification: gate inferred from a post pair
// ============================================================
//
// These three mirror SearchTarget / ApproachTarget / FinalAlignTarget one for
// one, but steer on the midpoint of two post-shaped detections instead of on a
// single box centre. Use them when the gate's top bar is too close to the
// surface to survive detection; use the originals when the whole gate is
// reliably boxed. Nothing here touches the original three.

class SearchGateByPosts : public BT::ActionNodeBase {
public:
    SearchGateByPosts(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    int sweep_direction_ = 1;
};

class ApproachGateByPosts : public BT::ActionNodeBase {
public:
    ApproachGateByPosts(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    int lost_frames_ = 0;
    int blind_frames_ = 0;
};

class AlignGateByPosts : public BT::ActionNodeBase {
public:
    AlignGateByPosts(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time align_start_time_;
    bool aligning_ = false;
    int lost_frames_ = 0;
};

// ============================================================
// New BT Nodes — Mission Control
// ============================================================

class WaitForStart : public BT::ActionNodeBase {
public:
    WaitForStart(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

// 全域時間預算守衛。ConditionNode，無副作用、不控制推進器，可以放在
// ReactiveFallback 底下每個 tick 重評而不會干擾任何動作節點。
//
// 兩種用法：
//   直接用   —— 「時間夠才進入這段可選任務」
//   包 Inverter —— 「時間不夠時觸發」，用來做截止上浮
class MissionTimeLeft : public BT::ConditionNode {
public:
    MissionTimeLeft(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class FinishMission : public BT::SyncActionNode {
public:
    FinishMission(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};
