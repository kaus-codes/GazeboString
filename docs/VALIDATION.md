# Validation record

Commands executed against the actual ARM64 project runtime:

```bash
./scripts/setup.sh
# OpenCV 4.6.0 / NumPy 1.26.4; CMake configured; suction/auditor library built.
docker exec gazebo-rgb-sorting-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash; python3 -m unittest discover -s /project/tests -v'
# Nine regression tests passed. Log: results/preflight/unit-tests-delivery.log.

docker exec gazebo-rgb-sorting-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash; python3 /project/src/run_integration.py --phase full --seed 11'
docker exec gazebo-rgb-sorting-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash; python3 /project/src/run_integration.py --phase full --seed 23 --gui --record'
docker exec gazebo-rgb-sorting-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash; ros2 launch /project/launch/sorting.launch.py seed:=47 gui:=false record:=false'

# Applied independently to each completed final run:
docker exec gazebo-rgb-sorting-dev python3 /project/src/acceptance.py /project/results/full-seed-11
docker exec gazebo-rgb-sorting-dev python3 /project/src/acceptance.py /project/results/full-seed-23
docker exec gazebo-rgb-sorting-dev python3 /project/src/acceptance.py /project/results/full-seed-47
```

Results are recorded in `results/CAMPAIGN.md`; acceptance requires actual RGB frames and all five colors, ineligible distractors, three physical shapes, calibrated localization, observed tracking motion, exact alternation, fourteen unique verified releases, no incorrect grasp, current final containment, actual joint movement, ordered timings and exports, continuously moving belt odometry, actual cup contacts, complete correct bin inventory, and clean owned-process shutdown.

Earlier independently executed gates verified the stock TrackController conveyor, physical movement of cubes/cylinders/spheres, actual ROS RGB and CameraInfo, camera localization against separate ground truth, real Panda joint displacement, MoveIt plan execution, stationary contact/lift/release/retention, and moving red-blue interception. Original failures and fixes remain in `results/` and `docs/TROUBLESHOOTING.md`.

The actual seed-23 X11 recording was probed with ffprobe: H.264, 1280×800, 4 fps, 601.5 seconds. Its sequence timing excludes startup and shutdown; the recording includes them. The noVNC browser inspection of the latest layout was blocked by automatic approval review because workspace credits were depleted. This document does not claim visual acceptance of that layout. Review it before presenting.

Core tests supplement actual Gazebo integration; they are not the basis for the fourteen-placement claim. Named final runs do not establish an unbiased reliability estimate over all possible layouts. Clutter and a clean Dockerfile rebuild remain unverified.

The seed-23 MP4 was fully decoded with `ffmpeg -v error -threads 1 -i /project/results/full-seed-23/actual_gazebo_screen.mp4 -f null -`, which exited 0 without errors. SHA-256: `b4194355b5a08f2b63281b82938efc2b32992f8d00d6c5d195c0df985f5a271f`.

All three run snapshots have identical hashes for the delivered critical controller, motion, vision, physical verifier, sequence validator, suction plugin, and control/perception configuration. Post-run additions affect reporting/documentation and explicitly displayed remaining/incorrect counters; the latter panel-only edit was syntax-checked but still awaits visual review.
