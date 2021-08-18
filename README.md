# stan-head
ROS nodes used for Stan's brain. Runs on Ubuntu 20.04 with ros Noetic. 

To build, install the full desktop version of ROS:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

- Clone and build in a ROS workspace src folder which can contain other packages, e.g.:
  cd ~/ros1_ws/src/
  clone https://github.com/david-beaudette/stan-head.git
  catkin_make install

- Add user to dialout (e.g. with user "ubuntu"):
  sudo adduser ubuntu dialout

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

- [Camera node] (https://github.com/UbiquityRobotics/raspicam_node) for RPi camera
  Clone in ~/ros1_ws/src/ 

- Wireless watcher for connection monitoring:
  sudo apt install ros-noetic-wireless-watcher

- Set hostname of RPi to "stanhead" and make it known to the UI computer.

- Create SSH keys and copy them on both sides:
  [Instructions](https://linuxize.com/post/how-to-set-up-ssh-keys-on-ubuntu-20-04/)

- Install services on the RPi:
  sudo cp ~/ros1_ws/install/share/stan_common/scripts/rpi_roscore.service /etc/systemd/system/rpi_roscore.service
  sudo cp ~/ros1_ws/install/share/stan_common/scripts/rpi_stan.service /etc/systemd/system/rpi_stan.service
  sudo systemctl daemon-reload
  sudo systemctl enable rpi_roscore
  -> eventually: sudo systemctl enable rpi_stan
  sudo systemctl start rpi_roscore
  -> eventually: sudo systemctl start rpi_stan
  
  