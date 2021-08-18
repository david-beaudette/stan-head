# stan-head
ROS nodes used for Stan's brain. Runs on Ubuntu 20.04 with ros Noetic. 

Should be placed in a ROS workspace src folder which can contain other packages, e.g.:
~/ros1_ws/src/stan-head/src/sermonizer/

To build, install the full desktop version of ROS:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

- Add user to dialout (e.g. with user "david"):
  sudo adduser david dialout

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
  Clone in stan-head/../ to build alongside stan packages.

- Wireless watcher for connection monitoring:
  sudo apt install ros-noetic-wireless-watcher

- Set hostname of RPi to "stanhead" and make it known to the UI computer.

- Create SSH keys and copy them on both sides:
  [Instructions](https://linuxize.com/post/how-to-set-up-ssh-keys-on-ubuntu-20-04/)