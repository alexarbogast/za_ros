# za_ros
**ROS integration for the Tormach ZA6 robot**

*DISCLAIMER: This is an UNOFFICIAL robot integration package. The author is not affliated with Tormach Inc..*

<p align="center">
<img src="https://user-images.githubusercontent.com/46149643/221460204-b7701307-9064-46bb-81d1-437f7d06e125.png" width=50% height=50%>
</p>

## Contents

- [Installation](#1)
- [running the demos](#2)
<br>
<a id='1'></a>

## Installation

Za_ros depends on the [cartesian_controllers](https://github.com/alexarbogast/cartesian_controllers) package for trajectory generation in some of the controllers. Follow the instructions at [cartesian_controllers](https://github.com/alexarbogast/cartesian_controllers) to install the package.

Clone the repository to your local workspace. If you do not already have the 
[za_description](https://github.com/alexarbogast/za_description/tree/e57f65c3f8eb0be88e7739a8b5162b4b3b875b15) package, you will need to clone recursively. 

```shell script
mkdir catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/alexarbogast/za_ros.git
```

Build your workspace
```shell script
catkin build
```

<br>
<a id='2'></a>

## Running the Demos
The *za_gazebo* package implements a hardware interface for the gazebo simulation. When launching the gazebo simualtion, the hardware interface provides automatic gravity compensation for the robot. 

Launch the Za6 gazebo simulation with the following command
```shell script
roslaunch za_gazebo za_robot.launch
```

If you would like to launch a controller with the robot, the *controller* parameter can be used to select the desired ros_controls controller.
```shell script
roslaunch za_gazebo za_robot.launch controller:=CONTROLLER_NAME
```
A few controller have been implemented in za_controllers.
- cartesian_velocity_controller
- cartesian_posvel_controller
- task_priority_controller
- cartesian_trajectory_controller 
- task_priority_trajectory_controller

The available controllers can be found in [sim_controllers.yaml](za_gazebo/config/sim_controllers.yaml). More information about the available controllers refer to [za_controller detailed description](za_controllers/README.md).

To actually execute a trajectory, you will need a node that feeds the controllers a setpoint. An example for executing linear trajectories with 4th order smoothing is provided at [kintrol](https://github.com/alexarbogast/kintrol.git). The kintrol package implements an online trajectory generator using [ruckig](https://github.com/pantor/ruckig).

### TrajectoryControllers

Run the trajectory controller demos. To use the task priority trajectory controller, for example:

```shell ros
roslaunch za_gazebo za_robot.launch controller:=task_priority_trajectory_controller
```

In a new shell (after sourcing *setup.bash*):
```shell script
rosrun za_controllers task_priority_trajectory_controller_test.py
```