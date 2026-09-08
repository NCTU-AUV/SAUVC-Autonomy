#include "orca_decision/world_model.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

WorldModel::WorldModel(rclcpp::Node* node, double velocity_decay, double max_vel, double timeout_sec)
    : node_(node), velocity_decay_(velocity_decay), max_velocity_clamp_(max_vel), perception_timeout_sec_(timeout_sec)
{
}

void WorldModel::updateFromPerception(const orca_interface::msg::PerceptionArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time now = node_->now();
    std::unordered_set<std::string> updated_labels;

    for (const auto& obj : msg->objects) {
        if (obj.is_stable) {
            updated_labels.insert(obj.label);
        }
    }

    if (!updated_labels.empty()) {
        tracked_objects_.erase(
            std::remove_if(
                tracked_objects_.begin(), tracked_objects_.end(),
                [&](const TrackedObject& tracked) {
                    return updated_labels.count(tracked.label) > 0;
                }),
            tracked_objects_.end());
    }

    for (const auto& obj : msg->objects) {
        if (!obj.is_stable) continue;

        TrackedObject tracked;
        tracked.label = obj.label;
        tracked.confidence = obj.confidence;
        tracked.distance = obj.distance;
        tracked.cx = obj.cx;
        tracked.cy = obj.cy;
        tracked.width = obj.width;
        tracked.height = obj.height;
        tracked.last_seen = now;

        // Project position to world frame
        Eigen::Vector3d pos_camera(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
        
        // For AUV, usually the camera frame is X right, Y down, Z forward. 
        // We will just store it directly for now, or map it using AUV orientation.
        // Assuming obj.pose is already relative to the AUV body frame or we apply orientation:
        Eigen::Vector3d pos_world = position_ + (orientation_ * pos_camera);
        tracked.position_world = pos_world;

        tracked_objects_.push_back(tracked);
    }

    removeStaleObjects();
}

void WorldModel::updateFromIMU(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time current_time = msg->header.stamp;

    if (!imu_initialized_) {
        last_imu_time_ = current_time;
        imu_initialized_ = true;
        // Assume start at 0,0,0 and yaw=0
        position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        return;
    }

    double dt = (current_time - last_imu_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        last_imu_time_ = current_time;
        return;
    }

    // Update orientation
    orientation_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    orientation_.normalize();

    // Rotate linear acceleration to world frame
    Eigen::Vector3d acc_body(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    
    Eigen::Vector3d acc_world = orientation_ * acc_body;

    // Remove gravity. The vehicle frame is down-positive (the control stack's
    // sink direction is +z), so the gravity vector is (0, 0, +9.81) and an
    // accelerometer at rest measures the specific force -g, i.e. z = -9.81.
    // Recovering coordinate acceleration is therefore a = f + g — an addition.
    // Subtracting here instead gave -19.62 m/s^2 at rest and the dead-reckoned
    // position free-fell (observed: z = -253 m after 20 s of a static mission).
    acc_world.z() += 9.81;

    // Integrate velocity
    velocity_ += acc_world * dt;

    // Velocity decay (friction/drag model), scaled by dt.
    //
    // Applying the factor once per message instead made the whole dead-reckoning
    // gain depend on the IMU rate: with constant acceleration the steady state
    // is a*dt*d/(1-d), i.e. inversely proportional to the publish rate, so at
    // 50 Hz the integrated position is double what it is at 100 Hz for the same
    // real motion, with nothing to indicate it. velocity_decay is documented as
    // a per-100-Hz-tick factor, so normalise against that reference rate and the
    // configured value keeps its meaning.
    constexpr double kDecayReferenceHz = 100.0;
    velocity_ *= std::pow(velocity_decay_, dt * kDecayReferenceHz);

    // Max velocity clamp
    if (velocity_.norm() > max_velocity_clamp_) {
        velocity_ = velocity_.normalized() * max_velocity_clamp_;
    }

    // Integrate position
    position_ += velocity_ * dt;

    last_imu_time_ = current_time;
}

std::optional<TrackedObject> WorldModel::getBestObject(const std::string& label) {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();

    std::optional<TrackedObject> best;
    float max_conf = -1.0f;

    for (const auto& obj : tracked_objects_) {
        if (obj.label == label && obj.confidence > max_conf) {
            best = obj;
            max_conf = obj.confidence;
        }
    }

    return best;
}

std::optional<TrackedObject> WorldModel::getObjectNearestImageCenter(
    const std::string& label, float center_x, float center_y) {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();

    std::optional<TrackedObject> best;
    float best_dist_sq = std::numeric_limits<float>::max();
    float best_conf = -1.0f;

    for (const auto& obj : tracked_objects_) {
        if (obj.label != label) {
            continue;
        }

        const float dx = obj.cx - center_x;
        const float dy = obj.cy - center_y;
        const float dist_sq = dx * dx + dy * dy;

        if (dist_sq < best_dist_sq ||
            (dist_sq == best_dist_sq && obj.confidence > best_conf)) {
            best = obj;
            best_dist_sq = dist_sq;
            best_conf = obj.confidence;
        }
    }

    return best;
}

std::optional<GatePostPair> WorldModel::getGatePostPair(
    const std::vector<std::string>& labels, const GatePostCriteria& criteria) {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();

    // 1. Keep only boxes that carry one of the accepted labels and are shaped
    //    like a post. A whole gate seen with its top bar is wider than it is
    //    tall, so the aspect test is also what stops this function from
    //    mistaking one full-gate detection for a post. Recorded runs put the
    //    flares — physically the same thin vertical cylinder as a gate post —
    //    at h/w 3 to 5, so the default threshold of 1.8 has real margin.
    std::vector<TrackedObject> posts;
    for (const auto& obj : tracked_objects_) {
        if (std::find(labels.begin(), labels.end(), obj.label) == labels.end()) continue;
        if (obj.width <= 0.0f || obj.height <= 0.0f) continue;
        if (obj.height / obj.width < criteria.min_aspect_ratio) continue;
        posts.push_back(obj);
    }
    if (posts.size() < 2) {
        return std::nullopt;
    }

    // 2. Sort by image x and only ever pair *adjacent* boxes. This is the one
    //    place the result is not literally "the two nearest posts": a pair with
    //    a third post between them is refused even if those two are the closest
    //    pair in the frame. That refusal is the point — steering at the midpoint
    //    of such a pair drives straight into whatever sits between them. With
    //    two gates in view there are four posts, and the outer two would
    //    otherwise be the widest, most gate-looking pair in the frame, aiming
    //    the vehicle at the open water between the gates. Adjacency rules that
    //    out without needing to know which post belongs to which gate.
    std::sort(posts.begin(), posts.end(),
              [](const TrackedObject& a, const TrackedObject& b) { return a.cx < b.cx; });

    std::optional<GatePostPair> best;
    float best_score = std::numeric_limits<float>::max();

    for (size_t i = 0; i + 1 < posts.size(); ++i) {
        const TrackedObject& left = posts[i];
        const TrackedObject& right = posts[i + 1];

        const float gap = right.cx - left.cx;
        if (gap < criteria.min_gap_px || gap > criteria.max_gap_px) continue;

        // Two posts of one gate are the same physical length at nearly the same
        // range, so they subtend nearly the same pixel height. A post paired
        // with something else — a far post, a flare, half a reflection — fails
        // here.
        const float taller = std::max(left.height, right.height);
        const float shorter = std::min(left.height, right.height);
        if (shorter <= 0.0f || taller / shorter > criteria.max_height_ratio) continue;

        // 3. Rank by range. Prefer resolved depth; fall back to apparent height,
        //    which is monotone in closeness for a fixed-length post. The two are
        //    not comparable to each other, so depth-resolved pairs always beat
        //    height-only ones rather than competing on a mixed scale.
        float distance = -1.0f;
        int valid = 0;
        float sum = 0.0f;
        if (left.distance > 0.0f)  { sum += left.distance;  ++valid; }
        if (right.distance > 0.0f) { sum += right.distance; ++valid; }
        if (valid > 0) {
            distance = sum / static_cast<float>(valid);
        }

        constexpr float kHeightOnlyPenalty = 1000.0f;
        const float score = (distance > 0.0f)
                                ? distance
                                : kHeightOnlyPenalty + 1.0f / std::max(taller, 1.0f);

        if (score < best_score) {
            best_score = score;
            GatePostPair pair;
            pair.left = left;
            pair.right = right;
            pair.center_cx = 0.5f * (left.cx + right.cx);
            pair.center_cy = 0.5f * (left.cy + right.cy);
            pair.gap_px = gap;
            pair.distance = distance;
            best = pair;
        }
    }

    return best;
}

std::vector<TrackedObject> WorldModel::getObjects() {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();
    return tracked_objects_;
}

Eigen::Vector3d WorldModel::getAUVPosition() {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_;
}

double WorldModel::getYaw() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Convert quaternion to Euler angles (yaw, pitch, roll)
    tf2::Quaternion q(orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w());
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

Eigen::Vector3d WorldModel::getAUVVelocity() {
    std::lock_guard<std::mutex> lock(mutex_);
    return velocity_;
}

void WorldModel::removeStaleObjects() {
    rclcpp::Time now = node_->now();
    tracked_objects_.erase(
        std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [&](const TrackedObject& obj) {
                return (now - obj.last_seen).seconds() > perception_timeout_sec_;
            }),
        tracked_objects_.end());
}
