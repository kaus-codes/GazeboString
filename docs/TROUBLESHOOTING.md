# Troubleshooting and observed failures

- **Docker unavailable:** start the installed Docker Desktop. The project never starts unrelated stopped containers. Multipass was inspected but its socket failed a TLS certificate check; no Multipass settings were changed.
- **Package downloads fail:** preserve apt logs in `results/preflight`. Retry the specific failed download after repository refresh completes. Do not run concurrent apt mutations. Signed official apt repositories are used; TLS checks are not disabled.
- **No camera processing despite images:** inspect actual Gazebo topics. The camera-info topic is `/sorting/camera_info`, not `/sorting/rgb/camera_info`. The first failed probe is retained.
- **OpenGL issues:** the container uses Xvfb, Mesa software rendering, and Ogre2. Inspect the run's `gazebo.log` and `gui.log`. The M1's Metal GPU is not directly exposed as an OpenGL GPU inside this Linux container.
- **No MoveIt execution:** check `moveit-simple-controller-manager` is installed. Planning can succeed without a usable execution plugin; that is not integration success.
- **No contact grasp:** inspect the converted SDF. SDFormat renames URDF collision elements. The builder resolves the actual suction collision name before spawning. The initial sensor-name failure is preserved.
- **Camera occlusion:** the initial overhead view lost selected tracks as the arm approached. The oblique view faces the robot from the other side of the belt. A stale track causes rejection and bounded recovery.
- **Front-lane IK failure:** the initial Panda hand collided with the front rail at the low wrist pose. The collision diagnostic recorded a 1.33 mm intersection. The longer vacuum adapter raises the wrist without disabling collision checks.
- **Missing feeder objects or duplicate names:** use the blocking create service and verify the new model through actual pose messages. Names are unique even after an uncertain request. The original feeder failure and the five-object physical spawn probe are preserved.
- **Experiment fails:** use `failures.jsonl`, controller logs, frame logs, and evaluation files. Do not increase success counters manually or weaken the fourteen-placement requirement.
- **Another run is active:** the harness holds `/project/.run.lock`. Stop the active project run normally with Ctrl-C; do not delete a live lock or launch a second simulator in the same partition.
- **Shutdown:** the Python harness owns subprocess groups and bounds SIGINT/TERM cleanup. An interrupted ROS launch may report its signal exit. Check for live owned processes before starting another run; defunct processes do not simulate physics.

A fresh Dockerfile build is provided for recovery, but distinguish a tested existing container from any untested rebuild. Never prune unrelated Docker images or volumes to make space.

Additional failures diagnosed during integration:

- ROS `Pose` defaults to quaternion `w=1`. Setting only `x=1` produced an invalid Cartesian orientation. The normalized downward pose helper and geometric regression now prevent this.
- A descending position goal can be physically blocked after successful suction contact. The controller now cancels on contact and verifies the subsequent lift, rather than forcing the lower endpoint.
- Zero-distance home commands can produce an empty trajectory. Ready joints are first checked from actual feedback.
- An RPC timeout does not prove an operation failed. Robot spawning, feeder registration, and controller activation now use observed state and unique/idempotent requests.
- Scene change messages are not freshness evidence for sleeping objects. The physics observer and complete final-bin evidence retain current transforms and containment checks.

- **Browser inspection unavailable:** the final noVNC inspection was blocked by automatic approval review because workspace credits were depleted. Restore credits before asking the agent to inspect that browser page, or inspect the GUI yourself. Do not describe the latest layout as visually verified. The simulation and recording are independent of the browser.

- **Seed-47 transport path tolerances:** two real executions aborted in TRANSPORT; one logged joint index 4 position error −0.155684 rad against the 0.15-rad limit. The controller preserved the failures, disabled suction through normal recovery, returned ready, and retried the same required color. It completed 14 verified placements in 16 attempts, with no extra bin occupant. These intermittent tracking failures remain a reliability limitation; the tolerance was not relaxed to hide them.
