# stan-head
ROS2 nodes used for Stan's brain. Runs on Ubuntu 20.04 with ros2 Galactic.

To build, install:
- [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [colcon](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)
- Diagnostic updater (sudo apt install ros-galactic-diagnostic-updater)
- Build tools:
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget