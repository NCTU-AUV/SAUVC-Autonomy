#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "orca_decision/world_model.hpp"
#include "orca_decision/wrench_adapter.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

struct DecisionContext {
    rclcpp::Node* node;
    std::shared_ptr<WorldModel> world_model;
    std::shared_ptr<WrenchControllerAdapter> wrench_adapter;

    // Some common state flags
    bool mission_started = false;
    rclcpp::Time mission_start_time;
    std::string current_flare_order = "";

    // State for DecisionStatus
    std::string current_action = "";
    std::string target_label = "";
    bool is_recovering = false;
    std::string camera_mode = "";
    std::string debug_msg = "";

    // Thresholds
    float align_yaw_threshold = 0.1f;
    float align_distance_threshold = 0.5f;

    // 決賽現場池深校正（見 ZoneTargetDepth，world_model.cpp 旁邊的
    // decision_node.cpp 實作）。資格賽三棵樹完全不讀這些欄位 —— SetDepth 只有
    // 走 zone= port 才會碰到它們，資格賽的 SetDepth 全部用的是舊的 depth=
    // 字面值路徑。
    //
    // pool_depth_*_m：操作員在網頁上輸入的現場池深（三個輸入格：
    // gate / drop / flare）。
    // zone_offset_*_m：每個 zone 距水面的固定幾何常數，池深不變就不用動；
    // 換算式是 d = clamp(pool_depth − zone_offset, min_depth,
    // pool_depth − hull_bottom_margin)。
    //
    // drop 這個 zone 在 trees.xml 裡被用了兩次：放球前、撿球前，兩處要求的
    // 深度相同（0.30），所以共用同一個 zone。
    //
    // 預設值刻意讓換算結果等於目前 trees.xml 裡硬寫的決賽深度
    // （gate 0.7、drop 0.30、flare 0.9，假設池深分別是 1.60 / 1.20 / 1.60）
    // —— 網頁沒接上或現場忘了輸入時，行為與改動前完全一致。
    float pool_depth_gate_m = 1.60f;
    float pool_depth_drop_m = 1.20f;
    float pool_depth_flare_m = 1.60f;
    float zone_offset_gate_m = 0.90f;
    float zone_offset_drop_m = 0.90f;
    float zone_offset_flare_m = 0.70f;

    // 破水面保險（同 set_depth.cpp 的 kMinDepth）與離池底餘裕，ZoneTargetDepth
    // 的 clamp 上下界共用同一組常數，避免兩處各自維護一份數字。
    float min_depth = 0.35f;
    float hull_bottom_margin = 0.34f;

    // 最後一次由 zone= 設下的深度所屬的區。SetDepth 用 depth= 字面值（資格賽、
    // 上浮 depth="0.0"）時會清空這個欄位 —— 空字串代表「目前深度不是由某個
    // 區的池深換算出來的」，池深參數變更回呼看到空字串就不會重發深度指令。
    std::string current_depth_zone = "";

    // Publishers needed by BT nodes
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_depth_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hand_pub;
};

// 由現場池深換算某任務區的目標深度。SetDepth（zone= 分支）與 decision_node 的
// 池深參數回呼共用同一份實作，避免兩處算出不同數字。zone 只認
// "gate"/"drop"/"flare"；其他字串回傳 false，呼叫端要處理。
bool ZoneTargetDepth(const DecisionContext& ctx, const std::string& zone,
                      float* out_depth);
