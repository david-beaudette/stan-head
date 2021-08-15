# stan-head
ROS nodes used for Stan's brain. Runs on Ubuntu 20.04 with ros Noetic.

To build, install:
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

- Image stuff for RPi camera
TBW
