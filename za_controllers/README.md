# za_controllers
**Custom controllers implemented for the Tormach Za6 Robot**

## cartesian_velocity_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_velocity_controller"
```
This is a simple inverse jacobian resolved rate motion controller. The psuedo inverse jacobian is used to map commanded cartesian velocities to joint velocities


$$\dot{q}=\mathbb{J}^\dagger\dot{x}_d$$


The controller requires a velocity joint interface from the running hardware interface.

## cartesian_posvel_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_posvel_controller"
```
This controller is similar to the cartesian velocity controller, but a position
value is tracked in addition to the cartesian velocity. 

$$\dot{q}=\mathbb{J}^\dagger\left(\begin{bmatrix} K_pe_p \\ K_oe_{qv} \end{bmatrix}+ \begin{bmatrix} \dot{p_d} \\ \omega_d\end{bmatrix}\right)$$

$$e_p=p_d-p \in \mathbb{R}^3$$

$$e_q=\begin{bmatrix}e_{qs} & e_{qv}^T\end{bmatrix}^T=q_d \otimes q^{-1}$$

The controller requires a velocity joint interface from the running hardware interface

## task_priority_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_posvel_controller"
```
This controller attempts to achieve two tasks. The primary task is tracking a position $p \in \mathbb{R}^3$ and aligning the z-axis of the end effector $\hat{z}_e$ with a desired axis $\hat{z}_d$. This primary task will use 5 out of 6 of the robots DOFs. The other DOF (i.e. the rotation about $\hat{z}_e$) is used to achieve the second task of maximizing the manipulability of the robot.

We first find the hessian matrix of the robot $H$. We then use the hessian to find the gradient of the manipulability (or the manipulability jacobian).

manipulability
$$m=\sqrt{det(J(q)J(q)^T)}$$

$$\nabla{}m=\frac{\partial{m}}{\partial{q}}=\begin{bmatrix}m\cdot{}vec(JH_1^T)^T\cdot{}vec((JJ^T)^{-1}) \\ m\cdot{}vec(JH_2^T)^T\cdot{}vec((JJ^T)^{-1}) \\ \vdots \\ m\cdot{}vec(JH_n^T)^T\cdot{}vec((JJ^T)^{-1}) \end{bmatrix}$$

where vec is the column-wise matrix mapping $vec(.) : \mathbb{R}^{a\times{}b}\rightarrow{} \mathbb{R}^{ab}$. We can then project this gradient into the task space of the robot using the jacobian. 

$$\dot{x}_{m} = \begin{bmatrix} \dot{p}_{x,m} & \dot{p}_{y,m} & \dot{p}_{z,m} & \omega{}_{x,m} & \omega{}_{y,m} & \omega{}_{z,m} \end{bmatrix}^T = K_r \cdot J \nabla m$$

Projecting this onto the nullspace of the primary task requirements is then just a matter of zeroing the first five components such that

$$\dot{x}_{m,d} = Kr \cdot \begin{bmatrix} 0, 0, 0, 0, 0, \omega{}_{z,m}\end{bmatrix}^T$$


Finally, we put the combined control signal back through the psuedo inverse mapping to find output command.

$$\dot{q}=\mathbb{J}^\dagger\left(\begin{bmatrix} K_pe_p \\ K_oe_{qv} \end{bmatrix}+ \begin{bmatrix} \dot{p_d} \\ 0\end{bmatrix} + \dot{x}_{m,d}\right)$$

$$e_p=p_d-p \in \mathbb{R}^3$$

$$e_q=\begin{bmatrix}e_{qs} & e_{qv}^T\end{bmatrix}^T=q_d \otimes q^{-1}$$

This controller is desirable for applications where only one direction of the tools orientation needs to be specified. Examples of these applications include wire-arc additive manufacturing (WAAM), painting, and grinding applications to name a few. 

Since the manipulability is always being maximized, it is easy to command cartesian positions throught the entirety of the robots workspace without every having to specify an orientation.

The controller requires a velocity joint interface from the running hardware interface

## References
This work would not be possible without the following resources. A big thank you for their contributions and detailed explanations. 

```
@article{haviland2020mmc,
  title={A purely-reactive manipulability-maximising motion controller},
  author={Haviland, J and Corke, P},
  journal={arXiv preprint arXiv:2002.11901},
  year={2020}
}

@article{haviland2020systematic,
  title={A systematic approach to computing the manipulator Jacobian and Hessian using the elementary transform sequence},
  author={Haviland, Jesse and Corke, Peter},
  journal={arXiv preprint arXiv:2010.08696},
  year={2020}
}

@article{Lizarralde2022,
   author = {Nicolas Lizarralde and Fernando Coutinho and Fernando Lizarralde},
   issn = {2377-3766},
   issue = {4},
   journal = {IEEE Robotics and Automation Letters},
   pages = {9675-9682},
   title = {Online Coordinated Motion Control of a Redundant Robotic Wire Arc Additive Manufacturing System},
   volume = {7},
   year = {2022},
}
```