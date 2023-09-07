# za_ros
**ROS integration for the Tormach ZA6 robot**

*DISCLAIMER: This is an UNOFFICIAL robot integration package. The author is not affliated with Tormach Inc..*

<p align="center">
<img src="https://user-images.githubusercontent.com/46149643/221460204-b7701307-9064-46bb-81d1-437f7d06e125.png" width=50% height=50%>
</p>

## Contents

- [Installation](#1)
- [Dependencies](#2)
- [Running the Robot](#3)

<a id='1'></a>

## Installation

Clone the repository to your local workspace. If you do not already have the 
[za_description](https://github.com/alexarbogast/za_description/tree/e57f65c3f8eb0be88e7739a8b5162b4b3b875b15) package, you will need to clone recursively. 

```shell
mkdir catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/alexarbogast/za_ros.git
```

Build your workspace
```shell
catkin build
```

<a id='2'></a>

## Dependencies
Running the **physical robot** requires the `hal_hw_interface` hardware interface.
Users should follow the instructions at
[hal_ros_control](https://github.com/tormach/hal_ros_control) for installation.

An easier route is to use a docker container from the image provided with the 
Za6 control computer. See [za_docker](https://github.com/alexarbogast/za_docker)
for a Dockerfile and scripts that facilitate ROS development for the Za6.

If you are using this package for **simulation purposes only**, you will only
need
[ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate)
and [ros_controllers](https://github.com/ros-controls/ros_controllers) to get
started.

If you do not have these packages:
```shell
sudo apt update
sudo apt install ros-noetic-ros-control-boilerplate
sudo apt install ros-noetic-ros-controllers
```

<a id='3'></a>

## Running the Robot
The `za_robot` package contains a set of launch files for running 
the Za6. See the [README.md](./za_robot/README.md) for usage details.
