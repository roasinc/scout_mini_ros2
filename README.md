# scout_mini_ros2
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0/)
[![ubuntu20](https://img.shields.io/badge/-UBUNTU%2020%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/)
[![humble](https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Overview
ROS2 packages for Scout Mini

Manual
[Specification](https://roas.co.kr/scout-mini/)

## Installation
```
cd ~/colcon_ws/src
git clone https://github.com/roasinc/scout_mini_ros2.git

cd ~/colcon_ws/
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage
```
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_mini_base base.launch
```