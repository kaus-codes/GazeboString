# Measurement definitions

All numbers are collected from real execution. Controller events use Linux monotonic nanoseconds. Gazebo `/clock` supplies simulation seconds. These clocks are never subtracted from one another.

| Metric | Actual boundaries |
|---|---|
| Pixel-processing latency | OpenCV segmentation, morphology, contours, and calibrated localization; excludes ROS transport, display, and file export |
| Detection latency | Observed physical footprint entry into the calibrated region to first valid camera-track detection |
| Color confirmation | First track detection to its third confirming camera observation |
| Detection-to-selection | First track detection to actual target selection; includes waiting for the alternating sequence |
| Initial grasp planning | Initial grasp IK through validated approach plan |
| All motion-planning latency | Sum of measured IK, OMPL, and Cartesian service round trips during the operation |
| Reach and grasp | First observed joint movement to physical attachment confirmation |
| Grasp and lift | Attachment confirmation to verified lift of the same object |
| Transport | Lift confirmation to release command, including transport planning |
| Placement | Release to settled, geometrically contained object verification |
| Operation | Target selection to verified placement |
| Pair | Previous blue placement (or experiment start) to the next blue placement |
| Seven-pair sequence | Ready system before autonomous selection to the fourteenth verified placement; reported as successful only after the final retention/inventory audit passes |

Initialization, the final return to ready, report generation, and process cleanup are outside the sorting-sequence interval. Intermediate ready motions and waiting for colors are included. The dashboard freezes sorting elapsed time at the last required placement.

The evaluator samples physical poses every 50 simulated milliseconds, including sleeping bodies. Region entry means the first observed intersection of the oriented primitive's footprint with the calibrated detection region. Initial objects already inside are left-censored. Raw negative latency values are retained and excluded from aggregates as boundary-alignment uncertainty.

`events.jsonl` is the append-only controller record. `events.csv/json` are its exports. `full_events.csv/json` join actual camera and evaluator boundaries with physical object identities; censored boundaries remain null. `planner_calls.jsonl` separates service round-trip latency from OMPL's own reported planning time. `frame_processing.csv` contains real per-frame measurements.

The final approach intentionally cancels its trajectory on observed suction contact. A canceled approach is not a placement success: attachment, lift, release, settled containment, exact alternation, and the final bin inventory are checked separately. Raw contacts and the cancellation result are preserved.

Reports distinguish attempted picks, actual grasp starts, confirmed grasps, verified placements, failed attempts, and incorrect colors. No missing value is replaced by a manufactured timing. Statistics describe the measured runs on this M1/software-rendering environment, not universal robot performance.
