FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:1 GZ_PARTITION=gazebo_rgb_sorting
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' > /etc/apt/sources.list.d/gazebo.list && \
    echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-sim8-dev python3-gz-transport13 python3-gz-msgs10 \
      python3-opencv python3-numpy python3-matplotlib python3-yaml python3-pil ffmpeg \
      build-essential cmake xvfb x11vnc novnc websockify libgl1-mesa-dri \
      ros-jazzy-ros-base ros-jazzy-robot-state-publisher ros-jazzy-ros-gz-bridge \
      ros-jazzy-cv-bridge ros-jazzy-gz-ros2-control ros-jazzy-xacro \
      ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-kinematics \
      ros-jazzy-moveit-planners-ompl ros-jazzy-moveit-simple-controller-manager \
      ros-jazzy-moveit-resources-panda-moveit-config \
      ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /project
CMD ["sleep", "infinity"]
