# orca_decision

Autonomous mission decision system for the Orca AUV, built on **BehaviorTree.CPP v3**.

The node subscribes to perception output (`orca_interface/PerceptionArray`) and IMU data, maintains a **World Model** of tracked objects, executes a configurable **Behavior Tree** for mission orchestration, and outputs `geometry_msgs/Wrench` commands for thrust allocation.

## Architecture

```
/orca/perception_array ──► ┌──────────────┐
                           │              │ ──► /orca_auv/control/wrench_sources/decision  (Wrench)
/orca_auv/sensors/imu  ──► │ decision_node│ ──► /orca_auv/control/targets/depth_m          (Float64)
                           │              │ ──► /orca/decision/status                      (DecisionStatus)
/orca/decision/start   ──► │              │ ──► /orca/decision/status_json                 (String, JSON)
                           └──────────────┘ ──► /orca/decision/camera_mode                 (String)
```

### Core Components

| Component | File | Role |
|-----------|------|------|
| **DecisionNode** | `src/decision_node.cpp` | ROS 2 node — loads BT, subscribes to sensors, publishes wrench at 50 Hz, ticks BT at 10 Hz. |
| **WorldModel** | `src/world_model.cpp` | Thread-safe tracker: fuses IMU dead-reckoning with perception to maintain object positions in a world frame. Stale objects are removed after `perception_timeout_sec`. |
| **WrenchAdapter** | `src/wrench_adapter.cpp` | Low-pass filtered wrench generator: converts `MotionCommand` (surge/sway/heave/yaw) to `geometry_msgs/Wrench` with configurable gains `k_surge`, `k_sway`, `k_yaw`. |

### Behavior Tree Nodes

| Node | Type | Description |
|------|------|-------------|
| `SearchTarget` | Action | Rotate to search for a target label. |
| `ApproachTarget` | Action | Approach a detected target until within a specified distance. |
| `FinalAlignTarget` | Action | Fine yaw+lateral alignment to the target. |
| `BlindForward` | Action | Drive forward for a fixed duration with optional heading lock. |
| `TurnToYaw` | Action | Turn by a relative yaw offset in degrees. |
| `SetCamera` | Action | Switch the active camera mode (`realsense` / `usb`). |
| `SetDepth` | Action | Publish a desired depth target. |
| `AvoidObstacle` | Condition/Action | Reactive obstacle avoidance with sticky cooldown. |
| `SearchGateByPosts` | Action | Rotate to search until two post-shaped boxes form a gate pair. |
| `ApproachGateByPosts` | Action | Approach the midpoint of a post pair; falls back to post separation when depth fails. |
| `AlignGateByPosts` | Action | Fine yaw alignment onto the midpoint of a post pair. |

All three take a `label` port holding a **comma-separated** list, and the two posts of a pair
need not share a label — `label="gate"` for a 1-class model, or
`label="red_flare,blue_flare,yellow_flare,orange_flare"` to pick the gate out of whichever
flare colours the 7-class model assigns to its posts. Colour classes are unreliable under
uneven underwater lighting, which is exactly why a pair may be mixed.

## Configuration

### Parameters (`config/decision_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `k_surge` | 1.0 | Surge force gain |
| `k_sway` | 1.0 | Sway force gain |
| `k_yaw` | 0.5 | Yaw torque gain |
| `motion_lowpass_alpha` | 0.2 | Low-pass filter smoothing factor |
| `max_velocity_clamp` | 1.5 | Max estimated velocity (m/s) |
| `perception_timeout_sec` | 1.0 | Drop tracked objects unseen for this long |
| `velocity_decay` | 0.95 | Per-step velocity decay (drag model) |
| `tree_xml_file` | `config/trees.xml` | Path to BehaviorTree XML |
| `main_tree_id` | `FinalMission` | Root tree to execute |
| `align_yaw_threshold` | 0.1 | Yaw alignment tolerance (rad) |
| `align_distance_threshold` | 0.5 | Distance alignment tolerance (m) |
| `status_json_rate_hz` | 5.0 | Rate of the JSON status mirror; 0 disables it |
| `gate_post_min_aspect_ratio` | 1.8 | height/width a box must exceed to count as a gate post |
| `gate_post_min_gap_px` | 40.0 | Minimum post separation for a valid pair |
| `gate_post_max_gap_px` | 520.0 | Maximum post separation (rejects posts of two different gates) |
| `gate_post_max_height_ratio` | 1.8 | Max taller/shorter box-height ratio within a pair |
| `gate_posts_success_gap_px` | 230.0 | Post separation standing in for `distance` when depth is unresolved |

### Mission Trees (`config/trees.xml`)

- **QualificationMission** — Submerge → pass gate → U-turn → pass gate again → surface.
- **QualificationMissionBlindReturn** — Same outbound leg, but after the U-turn it blind-runs
  straight back through the gate instead of re-acquiring it. Tune the return `BlindForward`
  duration first; it has to cover the outbound blind run plus the 3 m approach standoff.
- **FinalMission** — Extended mission with obstacle avoidance.
- **PassGateProcedure** — Reusable sub-tree: search → approach → align → blind forward.
- **QualificationMissionGateOrFlares** — Whole-gate detection first; if it fails to acquire
  within 90 s, falls back to pairing post-shaped boxes. Then U-turn, blind return, surface.
  Works under either model: the fallback accepts `gate` (1-class) and every flare colour
  (7-class) in one label list, so the same tree covers both.
- **PassGateWithPostPairFallbackProcedure** — `AcquireGateWholeBox` under a 90 s cap, falling
  back to `AcquireGateByPostPair`, then a blind run through. The cap is load-bearing:
  `SearchTarget` returns RUNNING forever when it sees nothing, so without it the fallback
  branch is unreachable.

## Launch

```bash
ros2 launch orca_decision autonomy.launch.py
```

`autonomy.launch.py` starts the perception pipeline alongside the decision node.
`decision.launch.py` on its own gives you the BehaviorTree with nothing publishing
`/orca/perception_array` — the world model stays empty, every search/approach node
runs to its timeout, and the node graph still looks healthy. Use it only to debug
the tree in isolation.

The launch file remaps everything that crosses into the control stack onto the
vehicle namespace (`$ns` below, from the `namespace` argument, defaulting to
`ORCA_NAMESPACE`):

| Internal Topic | Remapped To | Why |
|----------------|-------------|-----|
| `/orca/imu/data` | `/$ns/sensors/imu` | Attitude source. Without it the world model never leaves its construction pose. |
| `/orca/decision/wrench` | `/$ns/control/wrench_sources/decision` | Motion commands onto the wrench bus. |
| `/orca/decision/desired_depth` | `/$ns/control/targets/depth_m` | Depth setpoint for the control stack's PID. |
| `/orca/decision/hand` | `/$ns/actuators/electromagnet/enabled` | Ball electromagnet, driven by `GrabBall`/`DropBall`. |

`/orca/decision/arm` (`ExtendArm`/`RetractArm`) is **not** remapped — the control
stack has no arm actuator topic, so those two nodes are still no-ops.

### Start a Mission

```bash
ros2 topic pub --once /orca/decision/start_mission std_msgs/msg/Bool 'data: true'
```

### Monitor

```bash
# Decision status (mission phase, current action, target lock, etc.)
ros2 topic echo /orca/decision/status

# Same fields as JSON. Published whether or not the mission is running, so it
# also distinguishes "not started" and "finished" from a dead node — which the
# DecisionStatus topic above cannot, since it goes silent when the tree stops.
# This is what the control stack's Web GUI shows in its Mission panel.
ros2 topic echo --full-length /orca/decision/status_json

# Wrench output
ros2 topic echo /orca_auv/control/wrench_sources/decision
```

## Dependencies

- `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`
- `orca_interface` (custom messages)
- `behaviortree_cpp_v3`
- `tf2`
