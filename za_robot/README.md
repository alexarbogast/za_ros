# Za Robot
This package provides configuration and launch files for running the 
Tormach Za6 robot in **simulation** and on the **physical hardware**.

The Za6 robot comes with a control computer that runs ROS in a docker container.
The remainder of this README provides insight and instructions for running the
Za6 on either the provided control computer or in simulation on your own
machine. If you desire running the packages on the physical robot, consider
using the docker image provided at
[za_docker](https://github.com/alexarbogast/za_docker).

## Running the Robot

Before running the robot, decide which `hardware_interface` configuration is
appropriate by consulting the [README.md](../za_hardware/README.md) in 
`za_hardware`. 

The [za_robot.launch](./launch/za_robot.launch) file will launch the hardware
interface and controllers. 

```shell
roslaunch za_robot za_robot.launch [hardware:=HARDWARE] [sim:=SIM] 

- HARDWARE: one of 'sim', 'hal'
- SIM: one of 'true', 'false'
```
By default, the launch file will use the `sim_hw_interface`. To use the
`hal_hw_interface`, pass the hardware option as **hal**. The hal_hw_interface
will default to sim mode. To run on the real hardware, pass the hardware option
as **hal** and the sim option as **false**. 

The SIM option does nothing if the HARDWARE option is passed as **sim**.

## Motion planning with MoveIt

To launch the rviz interactive motion planning enviornment, open another
terminal. 

```shell
roslaunch za_robot moveit_planning.launch
```
This will open rviz and allow you to plan and execute motion with the Za6 using
the interactive MoveIt plugin. This can be used on a host computer to send
trajectories to the controllers launched in the previous step. Alternatively,
you can use X11 port forwarding in the docker container to see the rviz GUI.
