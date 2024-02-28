## ROS1 Noetic: Raspberry Pi 4 & Ubuntu 20.04 in VirtualBox
### Raspberry Pi 4:
* **Step 1** : Install OS [Debian/Raspbian Buster](https://downloads.raspberrypi.com/raspios_oldstable_armhf/images/raspios_oldstable_armhf-2023-05-03/) on Raspberry Pi 4
* **Step 2** : Add the official ROS Debian repo to the OS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
