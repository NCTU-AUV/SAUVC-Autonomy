#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

static double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

BlindForward::BlindForward(const std::string &name,
                           const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList BlindForward::providedPorts() {
  return {BT::InputPort<double>("duration"),
          BT::InputPort<bool>("heading_lock")};
}

BT::NodeStatus BlindForward::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  double duration;
  bool heading_lock;
  if (!getInput<double>("duration", duration) ||
      !getInput<bool>("heading_lock", heading_lock)) {
    throw BT::RuntimeError("missing required inputs");
  }

  ctx_->current_action = name();
  ctx_->target_label = "";
  ctx_->debug_msg = "Blind forward";

  if (!started_) {
    started_ = true;
    start_time_ = ctx_->node->now();
    // 航向只在整段盲走的最開頭鎖一次，被 halt 打斷後重新進來不再取。
    if (heading_lock && !yaw_locked_) {
      target_yaw_ = ctx_->world_model->getYaw();
      yaw_locked_ = true;
    }
  }

  // 累計已走時間，而不是每次重新進來就從 0 起算。
  //
  // 這個節點會被 halt：FinalMission 把過門子樹包在
  // ReactiveFallback[AvoidObstacle, PassGateFinalsProcedure] 底下，避障一旦觸
  // 發就 haltChildren() 整棵子樹，盲走跟著被中止。原本 halt() 只做
  // started_ = false，下一 tick 重新取 start_time_，等於「避障觸發幾次就多走
  // 幾趟完整的 duration」。決賽盲走 10 秒、艇速約 0.65 m/s，多一趟就是多 6.5 m
  // —— 門到對面池壁沒有那麼多空間，碰壁 −5 分／次、5 次自動 abort。
  auto elapsed = elapsed_before_halt_ + (ctx_->node->now() - start_time_).seconds();
  if (elapsed >= duration) {
    started_ = false;
    elapsed_before_halt_ = 0.0;
    yaw_locked_ = false;
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::SUCCESS;
  }

  MotionCommand cmd;
  cmd.surge = 25.0f; // Constant surge

  if (heading_lock) {
    double current_yaw = ctx_->world_model->getYaw();
    double error = normalizeAngle(target_yaw_ - current_yaw);
    cmd.yaw = 1.0f * error; // P control for heading
  }

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void BlindForward::halt() {
  // 把這一段已經走掉的時間收進累計裡再停。下次 tick 會從累計值接著算，而不是
  // 從 0 重來。整段跑完（SUCCESS）時才把累計歸零。
  if (started_ && ctx_) {
    elapsed_before_halt_ += (ctx_->node->now() - start_time_).seconds();
  }
  started_ = false;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
