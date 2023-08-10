# scout_mini_ros2
[![Licence](https://img.shields.io/badge/License-Apache_2.0-green.svg)](https://opensource.org/licenses/Apache-2.0/)
[![ubuntu22](https://img.shields.io/badge/-UBUNTU_22.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![humble](https://img.shields.io/badge/-HUMBLE-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Overview
ROS2 packages for Scout Mini

[Specification](https://roas.co.kr/scout-mini/)

## Installation

### Install packages
```
cd ~/colcon_ws/src
git clone https://github.com/roasinc/scout_mini_ros2.git
```

### Install dependencies
```
sudo apt install python3-vcstool
vcs import ~/colcon_ws/src < ~/colcon_ws/src/scout_mini_ros2/requirement.rosinstall
```

### Build
```
cd ~/colcon_ws/
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage
```
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_mini_base base_launch.py
```
