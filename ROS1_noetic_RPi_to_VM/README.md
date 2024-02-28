## ROS1 Noetic: Raspberry Pi 4 & Ubuntu 20.04 in VirtualBox
### Raspberry Pi 4:
* **Step 1** : Install OS [Debian/Raspbian Buster](https://downloads.raspberrypi.com/raspios_oldstable_armhf/images/raspios_oldstable_armhf-2023-05-03/) on Raspberry Pi 4
* **Step 2** : Add the official ROS Debian repo to the OS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
```
* **Step 3** : Add official ROS key which is not specific to Noetic but for all ROS distros
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
* **Step 4** : Pull all meta info of ROS Noetic packages
```
sudo apt update
```
* **Step 5** : Install build dependencies on Raspberry Pi 4
```
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
* **Step 6** : Set up ROS Noetic dependency sources/repos
```
sudo rosdep init
rosdep update
```
* **Step 7** : Fetch & Install ROS Noetic dependencies
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
sudo apt-get install -y python3-wstool (or sudo apt-get install python-wstool)
wstool init src noetic-ros_comm-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
```
> * ***Extra*** : It is recommenced to increase the swap space, which is used when the physical memory space on your Raspberry Pi is used up
> ```
> sudo dphys-swapfile swapoff
> sudoedit /etc/dphys-swapfile
> ```
> Edit the file to increase the swap space form 100 MB to 1024 MB (1 GB): **CONF_SWAPSIZE=1024**
> ```
> sudo dphys-swapfile setup
> sudo dphys-swapfile swapon
> free -m
> ```
> Check if the swap status: Swap Total about 1024

* **Step 8** : Compiling Noetic packages on Raspberry Pi 4
```
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```
* **Step 9** : Verify Noetic installation on Raspberry Pi 4
```
source /opt/ros/noetic/setup.bash
roscore
```
---
### Ubuntu 20.04 in VirtualBox:
* **Step 1** : Install OS [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/) in VirtualBox
* **Step 2** : 選擇此虛擬機->設定->網路->把預設的NAT改為橋接介面卡->選擇電腦連接的對應網卡
* **Step 3** : 查看虛擬機與樹莓派的的ip與hostname
```
ifconfg
hostname
```
* **Step 4.1** : 虛擬機中運行`sudoedit /etc/hosts`打開檔案，添加下面箭頭那兩行 (__注意：ip與hostname之間一定要用Tab鍵__)
```
127.0.0.1       localhost
127.0.1.1       eric-VirtualBox
192.168.43.216  eric-VirtualBox  <---
192.168.43.52   raspberrypi  <---

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```
* **Step 4.2** : 樹莓派中運行`sudo nano /etc/hosts`打開檔案，添加下面兩行 (__注意：ip與hostname之間一定要用Tab鍵__)
```
127.0.0.1       localhost
192.168.43.216  eric-VirtualBox  <---
192.168.43.52   raspberrypi  <---
::1             localhost ip6-localhost ip6-loopback
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters

127.0.1.1               raspberrypi
```
* **Step 5** : 雙方重啟網路
```
sudo /etc/init.d/networking restart
```
或
```
sudo /etc/init.d/network-manager restart
```
* **Step 6** : 雙方安裝Chrony
```
sudo apt-get install chrony
```
* **Step 7** : 雙方安裝SSH
```
sudo apt-get install openssh-server
```
* **Step 8.1** : 虛擬機查看樹莓派網路連接
```
ping raspberrypi
```
* **Step 8.2** : 樹莓派查看虛擬機網路連接
```
ping eric-VirtualBox
```
* **Step 9.1** : 虛擬機添加環境變量，運行`sudoedit ~/.bashrc`，在檔案最底添加以下幾行 (__主機為樹莓派__)
```
source /opt/ros/noetic/setup.bash
source ~/ros_catkin_ws/devel/setup.bash
export ROS_HOSTNAME=eric-VirtualBox
export ROS_MASTER_URI=http://raspberrypi:11311
```
* **Step 9.2** : 樹莓派添加環境變量，運行`sudo nano ~/.bashrc`，在檔案最底添加以下幾行 (__主機為樹莓派__)
```
source /opt/ros/noetic/setup.bash
source ~/ros_catkin_ws/devel/setup.bash
export ROS_HOSTNAME=raspberrypi
export ROS_MASTER_URI=http://raspberrypi:11311
```
* **Step 10** : 樹莓派運行publisher節點，虛擬機運行`rostopic list`即可查看到發布的節點名稱
