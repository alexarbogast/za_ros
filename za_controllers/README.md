# za_controllers
**Custom controllers implemented for the Tormach Za6 Robot**

## cartesian_velocity_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_velocity_controller"
```
This is a simple inverse jacobian resolved rate motion controller. The psuedo inverse jacobian is used to map commanded cartesian velocities to joint velocities


$$\dot{q}=\mathbf{J}^\dagger\dot{x}_d$$


The controller requires a velocity joint interface from the running hardware interface.

## cartesian_posvel_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_posvel_controller"
```
This controller is similar to the cartesian velocity controller, but a position
value is tracked in addition to the cartesian velocity. 

$$\dot{q}=\mathbf{J}^\dagger\left(\begin{bmatrix} K_pe_p \\ K_oe_{qv} \end{bmatrix}+ \begin{bmatrix} \dot{p_d} \\ \omega_d\end{bmatrix}\right)$$

$$e_p=p_d-p \in \mathbb{R}^3$$

$$e_q=\begin{bmatrix}e_{qs} & e_{qv}^T\end{bmatrix}^T=q_d \otimes q^{-1}$$

The controller requires a velocity joint interface from the running hardware interface

## task_priority_controller
```shell script
roslaunch za_gazebo za_robot.launch controller="cartesian_posvel_controller"
```
This controller attempts to achieve two tasks. The primary task is tracking a position $p \in \mathbb{R}^3$ and aligning the z-axis of the end effector $\hat{z}_e$ with a desired axis $\hat{z}_d$. This primary task will use 5 out of 6 of the robots DOFs. The other DOF (i.e. the rotation about $\hat{z}_e$) is used to achieve the second task of maximizing the manipulability of the robot.

We first find the hessian matrix of the robot $H$. We then use the hessian to find the gradient of the manipulability (or the manipulability jacobian).

$$m=\sqrt{det(\mathbf{J}(q)\mathbf{J}(q)^T)}$$

$$\nabla{}m=\frac{\partial{m}}{\partial{q}}=\begin{bmatrix}m\cdot{}vec(\mathbf{J}H_1^T)^T\cdot{}vec((\mathbf{J}\mathbf{J}^T)^{-1}) \\ m\cdot{}vec(\mathbf{J}H_2^T)^T\cdot{}vec((\mathbf{J}\mathbf{J}^T)^{-1}) \\ \vdots \\ m\cdot{}vec(\mathbf{J}H_n^T)^T\cdot{}vec((\mathbf{J}\mathbf{J}^T)^{-1}) \end{bmatrix}$$

where vec is the column-wise matrix mapping $vec(.) : \mathbb{R}^{a\times{}b}\rightarrow{} \mathbb{R}^{ab}$. We want to project the gradient of the manipulability into the nullspace of the primary task Jacobian. Take the primary task jacobian $J_p$ to be the first five rows of the base frame Jacobian.

$$\mathbf{J}=\begin{bmatrix}\mathbf{J}_p \\ \mathbf{J}_{\omega_{z}}\end{bmatrix}$$

We can then project this gradient into the task space of the robot using the nullspace projection $(I - \mathbf{J}_p^\dagger\mathbf{J}_p)$.

Finally, we put the combined control signal back through the psuedo inverse mapping to find output command.

$$\dot{q}=\mathbf{J}_p^\dagger\left(\begin{bmatrix} K_pe_p \\ K_oe_{qv_{x,y}} \end{bmatrix}+ \begin{bmatrix} \dot{p_d} \\ 0\end{bmatrix}\right) + (I - \mathbf{J}_p^\dagger\mathbf{J}_p)K_r\nabla{}m$$

$$\mathbf{J}_p \in \mathbb{R}^{5 \times 6}, \quad (I - \mathbf{J}_p^\dagger\mathbf{J}_p) \in \mathbb{R}^{6 \times 6}$$
$$e_p=p_d-p \in \mathbb{R}^3$$

$$e_q=\begin{bmatrix}e_{qs} & e_{qv_{x,y}}^T & e_{qv_{z}}\end{bmatrix}^T=q_d \otimes q^{-1}$$

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