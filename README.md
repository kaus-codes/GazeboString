# Gazebo RGB vision sorting

**Implementation status: PARTIALLY COMPLETE — standard-mode physical sorting is verified; final graphical visual review is pending.** Full runs use actual Gazebo physics, RGB perception, Panda joint control, and contact-gated suction. See [the verified campaign](results/CAMPAIGN.md), individual `acceptance.json` files, and preserved development failures. Each accepted run requires fourteen audited releases in exact red-blue order, followed by a fresh final retention and bin-inventory audit.

The actual GUI run was recorded to MP4. The latest noVNC layout still needs presenter visual review: automatic browser approval review could not run because workspace credits were depleted. Clutter mode and a fresh Dockerfile build remain unverified.

## Launch on this Mac

Start Docker Desktop, then:

```bash
cd ~/Documents/GazeboRGBSorting
./scripts/setup.sh
./scripts/run_demo.sh 11
```

Open [the localhost noVNC desktop](http://localhost:6082/vnc.html?autoconnect=true&resize=scale). The demonstration uses actual Gazebo 3D rendering, actual RGB pixels, and controller-published counters. The script records the real X11 desktop to `results/full-seed-11/actual_gazebo_screen.mp4` when `--record` is enabled. It does not render a synthetic demonstration video.

A full autonomous headless attempt and strict evidence checks:

```bash
./scripts/run_headless_test.sh full 11
```

Individual gates:

```bash
./scripts/test_core.sh
./scripts/run_headless_test.sh motion
./scripts/run_headless_test.sh planning
./scripts/run_headless_test.sh static
./scripts/run_headless_test.sh moving 11
./scripts/generate_report.sh 11
```

The complete ROS launch entry point inside the container is:

```bash
docker exec -it gazebo-rgb-sorting-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash; ros2 launch /project/launch/sorting.launch.py seed:=11 gui:=true record:=true'
```

The supported setup path reuses the tested project container or saved project image. The Dockerfile is a recovery recipe from official ARM64 Ubuntu/ROS/Gazebo repositories; a clean rebuild is not automatically equivalent to the tested package snapshot.

## What the system does

An oblique 640×480 RGB camera observes a continuously moving TrackController belt. OpenCV segments red, blue, green, yellow, and orange from actual HSV pixels. Calibrated rays and camera-info intrinsics localize nominal object surfaces. Timestamped tracks estimate axial motion; stale and unreachable tracks are rejected. Only the required red or blue color can be selected.

MoveIt KDL IK and OMPL plan motions against the belt, rails, bins, floor, and supports. A seven-joint Panda executes real controller trajectories. A narrow suction extension attaches only after actual contact, follows the moving end effector through a Gazebo joint, and releases through joint removal. Gravity and collisions govern the bin drop. The independent evaluator confirms lift, correct-color identity, release, settled containment, and final retention. A separate validator accepts exactly `R,B` repeated seven times.

Standard mode separates shapes into lanes to prevent translating cylinders/cubes from catching rolling spheres. Input colors are seeded and shuffled, including distractors. Clutter mode has denser initial placement and variable orientations; it is an additional unverified challenge. To generate it without claiming an accepted result:

```bash
docker exec gazebo-rgb-sorting-dev python3 /project/src/generate_world.py \
  --seed 23 --mode clutter --output /project/worlds/clutter.sdf
```

## Verified execution and evidence

| Seed | Mode | Placements / attempts | Failures | Wall seconds | Sim seconds |
|---|---|---|---|---|---|
| 11 | Headless | 14 / 14 | 0 | 291.221 | 222.098 |
| 23 | GUI + recording | 14 / 14 | 0 | 563.017 | 249.910 |
| 47 | Headless | 14 / 16 | 2 | 443.393 | 273.162 |

All three runs passed all 23 independent checks, with zero incorrect picks. The 42/44 placement-to-attempt ratio is 95.45%; it describes this final campaign only. Seed 47 recovered from two path-tolerance violations without changing the required color or counting a failed attempt. All fourteen final objects remained correctly contained. Earlier development failures are preserved.


The final delivery checks executed `scripts/setup.sh`, the nine core regression tests, `run_integration.py --phase full --seed 11`, and a full GUI/recording run with seed 23. The accepted third seed was exercised through `ros2 launch /project/launch/sorting.launch.py seed:=47`. The exact final results and independent acceptance are in [CAMPAIGN.md](results/CAMPAIGN.md).

- Actual recorded GUI run: `results/full-seed-23/actual_gazebo_screen.mp4` (H.264, 1280×800, 4 fps).
- Each accepted run: `summary.json`, `acceptance.json`, `aggregate.json`, `REPORT.md`, CSV/JSON event and operation exports, `timing_report.png`, and `pipeline_comparison.png`.
- Every run preserves its source hashes, source snapshot, generated SDF/URDF, configuration, actual camera tracks, native planner calls, contacts, belt odometry, and final physics evidence.
- `results/campaign.csv/json` summarizes named final runs and separately inventories historical summaries. The final campaign success rate does not erase earlier development failures.

To regenerate final campaign evidence after generating all three reports:

```bash
docker exec gazebo-rgb-sorting-dev python3 /project/src/campaign_report.py /project/results --seeds 11 23 47
```

## Configuration and files

- `config/perception.json`: configurable HSV ranges, morphology, and tracking settings.
- `config/workstation.json`: camera pose, conveyor speed/spacing, robot home, bins, grasp heights, and bounded experiment limits.
- `config/controllers.yaml`, `config/bridge.yaml`: actual controller and ROS-Gazebo interfaces.
- `src/generate_world.py`, `src/build_robot.py`: SDF world and packaged Panda/tool construction.
- `src/vision.py`, `src/vision_node.py`: pixel-only perception and tracking.
- `src/motion.py`, `src/sorter.py`: IK/planning/controller interface and explicit autonomous state machine.
- `plugins/Suction.cc`: physically contact-gated suction attachment.
- `src/physical_verifier.py`, `src/sequence.py`: independent physical and exact-sequence checks.
- `src/metrics.py`, `src/report.py`, `src/acceptance.py`: actual timestamps, exports, plots, and strict offline acceptance.
- `results/`: real execution records, failures, frame logs, reports, and package snapshot.

Each new run archives the previous directory for its phase/seed. It also saves its generated world, robot model, and configurations. A project lock prevents two experiments from sharing the Gazebo partition. Ctrl-C triggers bounded cleanup of owned subprocess groups.

See [architecture](docs/ARCHITECTURE.md), [exact environment](docs/ENVIRONMENT.md), [professor procedure](docs/DEMONSTRATION.md), and [troubleshooting with observed failures](docs/TROUBLESHOOTING.md).

## Measurement limits

Processing latency, simulated duration, and monotonic wall elapsed time are separate. Statistics describe verified operations; failed attempts remain in logs. Initial region-entry observations can be left-censored, and boundary-alignment uncertainty can make raw detection latencies negative; those values are flagged rather than fabricated. RGB-only top views do not reliably distinguish spheres from cylinders, so the display reports “ROUND” and localization uncertainty. The suction attachment is a documented simulation approximation, not a fluid seal or friction-only gripper model.
