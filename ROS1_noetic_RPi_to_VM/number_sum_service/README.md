# Simple Service Example
## Ubuntu 20.04 & Raspberry Pi 4:
* **Step 1** : Create a new package of simple service
```
cd ros_catkin_ws/src
catkin_create_pkg service_example std_msgs rospy roscpp
cd service_example
mkdir srv
cd srv
touch AddTwoInts.srv
```
Edit the `AddTwoInts.srv`
```
int64 a
int64 b
---
int64 sum
```

* **Step 2** : Add the official ROS Debian repo to the OS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
```
