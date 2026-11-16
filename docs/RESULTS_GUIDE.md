# Reading the actual evidence

Start with `results/CAMPAIGN.md`, then open each accepted run's `acceptance.json` and `REPORT.md`.

| Artifact in each full-seed directory | Meaning |
|---|---|
| summary.json | Controller result and exact physical placement sequence |
| acceptance.json | Independent offline acceptance results |
| aggregate.json | Full measured statistics in separate wall/simulation domains |
| operations.csv / operations.json | Per-verified-operation derived durations; failed attempt boundaries remain in events/failures logs |
| full_events.csv / full_events.json | Joined camera, selection, manipulation, and verification boundaries |
| frame_processing.csv | Actual frame-processing samples |
| timing_report.png | Per-object, red/blue, stage, and pair-duration plots |
| pipeline_comparison.png | Separate recognition, native planning, and manipulation intervals |
| final_bin_audit.json | Fresh complete geometric retention checks for all accepted objects |
| bin_inventory.json | Correct-color accepted occupants and any violations |
| raw_suction_contacts.json | Actual Gazebo cup-contact evidence |
| conveyor_odometry.json | Observed moving-belt command/odometry during picks |
| planner_calls.jsonl | Native IK/OMPL/Cartesian service times and results |
| cartesian_execution.jsonl | Actual contact-triggered cancellation/attachment results |
| camera_frames.jsonl | Actual RGB-derived detections, labels, tracks, and positions |
| camera-evaluation-only.json | Independent camera color/localization evaluation |
| artifacts/ and source_sha256.json | Generated world/robot/configuration and source snapshot |
| runtime/shutdown.json | Owned process exit/reaping evidence |

The seed-23 actual MP4 includes initialization and teardown, so its 601.5-second duration differs from the measured seven-pair sorting interval. Its codec is H.264, at 1280×800 and 4 fps. The browser could not inspect the latest noVNC layout because automatic approval review was unavailable with depleted credits; presenter review is still required.

Historical failed attempts remain under their original or `-archived-...` names. `campaign.json` inventories their summaries separately. Failed and partial runs must never be substituted for accepted fourteen-release evidence. Raw frames/poses can be temporally correlated observations and do not constitute independent statistical trials.
