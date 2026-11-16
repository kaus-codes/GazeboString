# Selected environment

The inspected host is a MacBookPro17,1 with Apple M1 (arm64), 8 CPU cores, 8 GB unified RAM, and an 8-core Metal-capable M1 GPU. macOS reports version 27.2, build 26B118; the kernel reports Darwin 27.2.0. Initial free storage was about 16 GiB, so no native ROS/Gazebo source build or large VM was attempted.

Docker Desktop was already installed but stopped. The engine was started and an existing ARM64 Ubuntu/Gazebo image was reused without changing that image or its original project. All project dependency installs were confined to `gazebo-rgb-sorting-dev`. Its limits are 3 GB RAM, four CPUs, and 256 MB shared memory. Existing unrelated stopped containers were not started, deleted, or modified.

Verified components:

| Component | Verified version |
|---|---|
| Docker engine / client | 29.6.2 |
| Container OS | Ubuntu 24.04.5 LTS, aarch64 |
| Gazebo Sim | 8.15.0 (Harmonic) |
| ROS 2 | Jazzy; ros-base 0.11.0 |
| ros_gz_bridge | 1.0.24 |
| gz_ros2_control | 1.2.20 |
| MoveIt move_group | 2.12.4 |
| MoveIt Resources Panda | 3.1.0 |
| OpenCV | 4.6.0 |
| NumPy | 1.26.4 |
| gz-transport Python bindings | 13.6.0 |
| gz-msgs Python bindings | 10.4.0 |

See `results/environment/packages.tsv` for exact Debian package revisions. Runtime/compiler details are also in the preflight logs. macOS Homebrew Python 3.13.5 lacked OpenCV and is not the ROS runtime.

ROS/Gazebo run inside ARM64 Linux. The Mac's native Metal GPU is not passed through as a Linux OpenGL device. Rendering uses Mesa software OpenGL with Xvfb; x11vnc and websockify expose a desktop only on localhost port 6082. The actual camera rendering and noVNC panel were exercised. Graphical recordings have different performance from headless runs.

Multipass was available but could not connect to its daemon due to a certificate verification failure. That installation was not repaired or altered because Docker already provided a viable isolated runtime.

Official compatibility and implementation references:

- [Gazebo's ROS installation matrix](https://gazebosim.org/docs/harmonic/ros_installation/): Jazzy and Harmonic.
- [MoveIt getting started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html): Jazzy on Ubuntu 24.04.
- [gz_ros2_control Jazzy documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html).
- [TrackController API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrackController.html) and the installed `conveyor.sdf` example.
- [Official MoveIt Resources Panda model](https://github.com/moveit/moveit_resources/tree/ros2/panda_description).

The Dockerfile provides a fresh-build recipe from official repositories. The exact tested local container and saved image are the preferred replay path; a clean network rebuild must be validated independently because repository packages can change.

## Saved runtime

The tested filesystem was committed locally as `gazebo-rgb-sorting:jazzy-harmonic`, image ID `sha256:58ba818f695cf9f434a6f7b2ef9f5d6d3d3b7d0a6fe9a2b3c2c8f094251d8a25`. A new disposable container from that image successfully imported ROS Python, OpenCV 4.6.0 and NumPy 1.26.4 and reported Gazebo 8.15.0. This is a dependency-image smoke test, not a fresh end-to-end rebuild. Project files/results remain in the host bind mount and are not baked into the dependency image.
