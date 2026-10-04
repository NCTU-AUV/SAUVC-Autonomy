#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <orca_interface/msg/perception_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>
#include <optional>
#include <unordered_map>

struct TrackedObject {
    std::string label;
    float confidence;
    Eigen::Vector3d position_world; // Position in world frame
    float distance; // Distance from AUV
    float cx;       // Camera frame cx
    float cy;       // Camera frame cy
    float width = 0.0f;
    float height = 0.0f;
    rclcpp::Time last_seen;
};

// Two adjacent post-shaped detections that together read as a gate.
//
// The gate's top bar sits 8 cm under the surface (see the sim's q_gate model
// and the arena spawner), so at competition depth the bar is frequently washed
// out and the detector stops boxing the whole structure. What it produces
// instead depends on the model: a 1-class qualification model can only call a
// lone post "gate", while the 7-class finals model may well reach for one of
// its flare classes, since a flare is also a thin vertical cylinder. Either way
// the pair is what lets the mission steer through the opening: the posts are
// the evidence, the gate centre is inferred.
//
// Which label the posts actually arrive under is therefore not something this
// code should assume — it is a caller-supplied list, so the same machinery
// serves "gate", any mix of flare colours, or a future dedicated post class.
struct GatePostPair {
    TrackedObject left;    // smaller cx
    TrackedObject right;   // larger cx
    float center_cx;       // midpoint of the two posts, in tensor pixels
    float center_cy;
    float gap_px;          // right.cx - left.cx; grows monotonically as we close in
    float distance;        // mean of whichever post distances resolved; -1.0 if neither did
};

// Shape gates for post-pair extraction. All of them come from ROS parameters —
// see decision_node's declarations — so they can be retuned without a rebuild.
struct GatePostCriteria {
    float min_aspect_ratio;   // height/width a box must exceed to count as a post
    float min_gap_px;         // reject pairs too close together to be a real opening
    float max_gap_px;         // reject pairs too far apart (posts of two different gates)
    float max_height_ratio;   // taller/shorter box height; two posts of one gate look alike
};

class WorldModel {
public:
    WorldModel(rclcpp::Node* node, double velocity_decay, double max_vel, double timeout_sec);

    void updateFromPerception(const orca_interface::msg::PerceptionArray::SharedPtr msg);
    void updateFromIMU(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::optional<TrackedObject> getBestObject(const std::string& label);
    std::optional<TrackedObject> getObjectNearestImageCenter(
        const std::string& label, float center_x, float center_y);

    // Nearest post pair drawn from `labels`, or nullopt if no two tracked boxes
    // satisfy `criteria`. "Nearest" is by resolved depth when available and by
    // apparent height otherwise — the arena spawns a gate on both sides of the
    // pool, so which pair we pick is a real choice, not a formality.
    //
    // The two posts need not share a label. That is deliberate: the flare
    // classes are split by colour, and uneven underwater lighting routinely
    // makes one post of a pair read as a different colour from the other, so a
    // same-label requirement would throw away exactly the pairs this is for.
    std::optional<GatePostPair> getGatePostPair(const std::vector<std::string>& labels,
                                                const GatePostCriteria& criteria);
    std::vector<TrackedObject> getObjects();

    Eigen::Vector3d getAUVPosition();
    double getYaw();
    Eigen::Vector3d getAUVVelocity();

private:
    rclcpp::Node* node_;
    std::mutex mutex_;

    // Config parameters
    double velocity_decay_;
    double max_velocity_clamp_;
    double perception_timeout_sec_;

    // AUV State
    Eigen::Vector3d position_{0.0, 0.0, 0.0};
    Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
    rclcpp::Time last_imu_time_;
    bool imu_initialized_ = false;

    // Tracked objects
    // Key: label (We can have multiple objects of same label, but we track the most recent/best ones.
    // For simplicity, we just keep a list and cull old ones.)
    std::vector<TrackedObject> tracked_objects_;

    void removeStaleObjects();
};
