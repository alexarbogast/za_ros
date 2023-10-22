# Za Hardware

The `za_hardware` package provides the hardware interface and peripheral
equipment configurations for the Tormach Za6 robot. 

## Hardware Interfaces
> A [hardware interface](https://github.com/ros-controls/ros_control/wiki/hardware_interface) 
is used to communicate between a [ros_control](https://github.com/ros-controls/ros_control)
controller and the hardware to be controlled. 

The `za_hardware` package provides two possible hardware interface
configurations for the Za6. The first hardware interface is a simulation
interface provided by
[ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate/tree/noetic-devel).
The second is the [hal_ros_control](https://github.com/tormach/hal_ros_control)
interface provided by Tormach. See the descriptions below for more information
regarding each hardware interface if you are unsure of which to use.  

### sim_hw_interface

This hardware interface is used for pure *simulation purposes only*. It
facilitates the process of testing motion planning and controllers on the robot.
It is identical to the actual hardware interface in every aspect **except** for
how data are read from and written to the hardware.

The `sim_hw_interface` can be run on any computer with ROS (and
[ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate))
without needing the Za6 control computer. If you do not want to bother with
setting up a docker container or
[machinekit-hal](https://github.com/machinekit/machinekit-hal), this is the
preferred route.

### hal_hw_interface

This hardware interface uses
[machinekit-hal](https://github.com/machinekit/machinekit-hal) to bring
real-time control to the Za6. The `hal_hw_interface` can be run in simulation
mode or on the real hardware. Simulation mode does not require a real-time
patched kernel and can be helpful in troubleshooting any docker or machinekit
configuration problems. 

When running on the real hardware, consult
[za_docker](https://github.com/alexarbogast/za_docker) for help in preparing 
the necessary ROS packages.
