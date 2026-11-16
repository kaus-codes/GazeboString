# Architecture and evidence boundaries

```mermaid
flowchart LR
    G[Gazebo RGB camera] --> B[ros_gz_bridge]
    B --> I[ROS sensor_msgs/Image and CameraInfo]
    I --> V[OpenCV HSV segmentation]
    V --> T[Pixel centroids and calibrated ray-plane projection]
    T --> K[Color-gated object tracking]
    K --> S[Strict alternating target selector]
    S --> M[MoveIt KDL IK and OMPL planning]
    M --> C[gz_ros2_control trajectory controller]
    C --> A[Gazebo Panda joints and suction contact]
    A --> P[Independent lift and settled-bin verification]
    P --> Q[Independent R B sequence validator]
    Q --> E[Actual state and timing recorder]
    E --> D[Live RGB and state panel]
```

`vision.py` and `vision_node.py` consume camera pixels and calibration only. They do not import the simulator evaluator, feeder, spawn manifest, or model names. Red segmentation has both low-hue and high-hue ranges. Green, yellow, and orange are detected for visualization but are ineligible for selection. Confidence is a saturation measure, not a calibrated probability.

The configured Gazebo camera pose supplies the extrinsics. Gazebo cameras view along local +X; ROS optical coordinates view along +Z with +X right and +Y down. `workstation_camera()` constructs that basis change and the configured rigid transform. The intrinsic matrix comes from the actual camera-info topic. Localization intersects the resulting world ray with a nominal object surface plane. Round-object height and partial occlusion remain sources of uncertainty; no depth camera or metadata-derived color is used.

Tracking uses timestamped position prediction, one-to-one gated association by color, velocity smoothing, hit confirmation, and track expiry. The controller checks freshness and reachability again during interception. A limited coast during tool occlusion is explicit and bounded. Successful object identities are established only after contact, independently of target selection.

## Physics and grasping

The conveyor uses Gazebo Harmonic's existing TrackController, which applies contact-surface motion. The belt mesh stays fixed; physical objects translate or roll under contact and friction. A sphere need not translate at belt speed. The world uses primitive object collision geometry, mass-consistent inertias, and full-height rails.

The robot is the packaged MoveIt Resources Panda with a fixed world anchor. Original open fingers are replaced by a narrow vacuum extension. The tool offset and flange heights are configured together to keep the wrist clear of the rails. The suction model uses a real Gazebo fixed attachment created only when the suction face's contact sensor reports contact with a dynamic `part_` body within a short spatial bound. No object poses are set during grasp, lift, transport, or placement. Disabling suction removes that joint; gravity and collision physics govern the release.

MoveIt receives collision geometry for the belt, rails, floor, legs, and bins. The normal approach and transport use OMPL and a real FollowJointTrajectory controller. The final descent uses a continuous MoveIt Cartesian path with collision checking, 5 mm sampling, joint-jump bounds, and interception prediction iterated against its native retimed duration. Contact triggers intentional trajectory cancellation, followed by independent attachment and lift checks. Dynamic clutter is not represented as a complete MoveIt collision scene; clutter mode is consequently an unverified challenge, not an accepted capability.

## Independent evaluation

`physical_verifier.py` is a separate evaluation channel. It reads Gazebo object poses and the evaluation-only spawn manifest to check grasp identity after attachment, object lift, ground-truth color agreement, and containment after release. That information never chooses a target or supplies a perception color. Successful placements require release, full oriented-primitive containment within the inner bin faces, and a stationary period. A final audit checks all accepted objects remain in their bins.

`sequence.py` accepts only physically verified placements in R,B order, rejects duplicates and incorrect bins, and requires fourteen placements for seven complete pairs. Recovery cannot advance the required color. Incorrect-color grasps invalidate a run.

## Time domains

Camera timestamps and physical state use Gazebo simulation time. Processing and interprocess event boundaries use Linux CLOCK_MONOTONIC through `perf_counter_ns` / `monotonic_ns` in the same VM. Never subtract a simulated timestamp from a wall timestamp. The experiment excludes process startup before readiness; operation durations start at actual target selection. Camera-frame processing latency is measured around OpenCV work, independently of simulated elapsed time.

Ground-truth region entry is observed separately. Initial objects already inside the region when monitoring begins are left-censored. Raw negative entry/detection differences caused by region-boundary localization uncertainty are retained and flagged; reports do not turn them into plausible positive latencies. This limitation must be mentioned when presenting detection latency.

## Final control and audit details

The approach uses a normalized downward quaternion, a grasp-seeded pregrasp configuration, and MoveIt's collision-checked Cartesian-path service. Target prediction is iterated against the newly time-parameterized trajectory until the interception endpoint converges. Actual suction contact cancels the descending trajectory before it forces the object against the belt. The robot returns to its configured ready joints between placements.

`StateAuditor` publishes current physics transforms every 50 simulated milliseconds, including unchanged/sleeping bodies. Final containment uses actual primitive dimensions and orientations against the bin's inner faces, with a 1 mm contact tolerance. The inventory also rejects incorrectly colored or unverified objects in either bin. A failed placement verification invalidates the run.

See [measurement definitions](METRICS.md) for the exact benchmark boundaries and censoring rules.
