#include <rclcpp/rclcpp.hpp>
#include "orca_decision/decision_node.hpp"
#include "orca_decision/behavior_tree_nodes.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<DecisionContext> ctx) {
    // ============================================================
    // Existing nodes
    // ============================================================
    factory.registerBuilder<SearchTarget>("SearchTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SearchTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<ApproachTarget>("ApproachTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<ApproachTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<FinalAlignTarget>("FinalAlignTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<FinalAlignTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<BlindForward>("BlindForward", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<BlindForward>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<TurnToYaw>("TurnToYaw", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<TurnToYaw>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<AvoidObstacle>("AvoidObstacle", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<AvoidObstacle>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SetCamera>("SetCamera", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SetCamera>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SetDepth>("SetDepth", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SetDepth>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Task 2: Target Acquisition
    // ============================================================
    factory.registerBuilder<MoveAboveTarget>("MoveAboveTarget",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<MoveAboveTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<CheckBottomClear>("CheckBottomClear",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<CheckBottomClear>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<DropBall>("DropBall",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<DropBall>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Task 4: Communication & Localization
    // ============================================================
    factory.registerBuilder<WaitForFlareOrder>("WaitForFlareOrder",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<WaitForFlareOrder>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<BumpFlare>("BumpFlare",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<BumpFlare>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SkipFlare>("SkipFlare",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SkipFlare>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Task 3: Target Reacquisition
    // ============================================================
    factory.registerBuilder<GoToPose>("GoToPose",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<GoToPose>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SearchBottomTarget>("SearchBottomTarget",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SearchBottomTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SpiralSearchBottom>("SpiralSearchBottom",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SpiralSearchBottom>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<ExtendArm>("ExtendArm",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<ExtendArm>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<GrabBall>("GrabBall",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<GrabBall>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<RetractArm>("RetractArm",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<RetractArm>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Qualification: gate inferred from a post pair
    // ============================================================
    factory.registerBuilder<SearchGateByPosts>("SearchGateByPosts",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SearchGateByPosts>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<ApproachGateByPosts>("ApproachGateByPosts",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<ApproachGateByPosts>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<AlignGateByPosts>("AlignGateByPosts",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<AlignGateByPosts>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Mission Control
    // ============================================================
    factory.registerBuilder<WaitForStart>("WaitForStart",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<WaitForStart>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<MissionTimeLeft>("MissionTimeLeft",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<MissionTimeLeft>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<FinishMission>("FinishMission",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<FinishMission>(name, config);
            node->setContext(ctx);
            return node;
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
