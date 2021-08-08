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

- Qt bindings (not sure if required)
cd /usr/bin
pip install PySide2

- On the RPi only
sudo nano /boot/firmware/config.txt
At the end of the file, add:
start_x = 1
Ctrl+o to save
Ctrl+x to exit
sudo reboot now
(wait)
sudo apt update
sudo apt upgrade

- Image stuff for RPi camera
mkdir -p ~/schnitzes/ros2_ws/src
cd ~/schnitzes/ros2_ws/src

git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git
git clone --branch ros2 https://github.com/ros-perception/vision_opencv.git
git clone --branch ros2 https://github.com/ros-perception/image_common.git
git clone --branch ros2 https://github.com/ros-perception/image_transport_plugins.git
cd ..

rosdep init
rosdep install --from-paths src -r -y