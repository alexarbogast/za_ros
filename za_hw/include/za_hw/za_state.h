#pragma once

#include <array>


namespace za
{
/**
 * Describes the robot's current mode.
 */
enum class RobotMode {
  kOther,
  kIdle,
  kMove,
  kUserStopped
};

struct RobotState
{
    /**
     * \f$^{O}T_{EE}\f$
     * Measured end effector pose in @ref o-frame "base frame".
     * Pose is represented as a 4x4 matrix in column-major format.
     */
    std::array<double, 16> O_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f${^OT_{EE}}_{d}\f$
     * Last desired end effector pose of motion generation in @ref o-frame "base frame".
     * Pose is represented as a 4x4 matrix in column-major format.
     */
    std::array<double, 16> O_T_EE_d{};  // NOLINT(readability-identifier-naming)
    
    /**
    * \f$^{F}T_{EE}\f$
    * End effector frame pose in flange frame.
    * Pose is represented as a 4x4 matrix in column-major format.
    *
    * @see F_T_NE
    * @see NE_T_EE
    * @see Robot for an explanation of the F, NE and EE frames.
    */
    std::array<double, 16> F_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
    * \f$^{F}T_{NE}\f$
    * Nominal end effector frame pose in flange frame.
    * Pose is represented as a 4x4 matrix in column-major format.
    *
    * @see F_T_EE
    * @see NE_T_EE
    * @see Robot for an explanation of the F, NE and EE frames.
    */
    std::array<double, 16> F_T_NE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{NE}T_{EE}\f$
     * End effector frame pose in nominal end effector frame.
     * Pose is represented as a 4x4 matrix in column-major format.
     *
     * @see Robot::setEE to change this frame.
     * @see F_T_EE
     * @see F_T_NE
     * @see Robot for an explanation of the F, NE and EE frames.
     */
    std::array<double, 16> NE_T_EE{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$m_{EE}\f$
     * Configured mass of the end effector.
     */
    double m_ee{};

    /**
     * \f$I_{EE}\f$
     * Configured rotational inertia matrix of the end effector load with respect to center of mass.
     */
    std::array<double, 9> I_ee{};  // NOLINT(readability-identifier-naming)

    /**
    * \f$^{F}x_{C_{EE}}\f$
    * Configured center of mass of the end effector load with respect to flange frame.
    */
    std::array<double, 3> F_x_Cee{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$m_{load}\f$
     * Configured mass of the external load.    
     */
    double m_load{};

    /**
     * \f$I_{load}\f$
     * Configured rotational inertia matrix of the external load with respect to center of mass.
     */
    std::array<double, 9> I_load{};  // NOLINT(readability-identifier-naming)

    /**
     * \f$^{F}x_{C_{load}}\f$
     * Configured center of mass of the external load with respect to flange frame.
     */
    std::array<double, 3> F_x_Cload{};  // NOLINT(readability-identifier-naming)

    /**
    * \f$I_{total}\f$
    * Combined rotational inertia matrix of the end effector load and the external load with respect
    * to the center of mass.
    */
    std::array<double, 9> I_total{};  // NOLINT(readability-identifier-naming)

    /**
    * \f$^{F}x_{C_{total}}\f$
    * Combined center of mass of the end effector load and the external load with respect to flange
    * frame.
    */
    std::array<double, 3> F_x_Ctotal{};  // NOLINT(readability-identifier-naming)

    /**
    * \f$m_{total}\f$
    * Sum of the mass of the end effector and the external load.
    */
    double m_total{};

    /**
     * \f$q\f$
     * Measured joint position. Unit: \f$[rad]\f$
     */
    std::array<double, 6> q{};

    /**
     * \f$q_d\f$
     * Desired joint position. Unit: \f$[rad]\f$
     */
    std::array<double, 6> q_d{};

    /**
     * \f$\dot{q}\f$
    * Measured joint velocity. Unit: \f$[\frac{rad}{s}]\f$
    */
    std::array<double, 6> dq{};
    
    /**
     * \f$\dot{q}_d\f$
    * Desired joint velocity. Unit: \f$[\frac{rad}{s}]\f$
    */
    std::array<double, 6> dq_d{};

    /**
    * \f$\ddot{q}_d\f$
    * Desired joint acceleration. Unit: \f$[\frac{rad}{s^2}]\f$
    */
    std::array<double, 6> ddq_d{};

    /**
    * \f$\theta\f$
    * Motor position. Unit: \f$[rad]\f$
    */
    std::array<double, 6> theta{};

    /**
    * \f$\dot{\theta}\f$
    * Motor velocity. Unit: \f$[\frac{rad}{s}]\f$
    */
    std::array<double, 6> dtheta{};

    /**
    * Indicates which contact level is activated in which joint. After contact disappears, value
    * turns to zero.
    *
    * @see Robot::setCollisionBehavior for setting sensitivity values.
    */
    std::array<double, 6> joint_contact{};

    /**
    * Indicates which contact level is activated in which joint. After contact disappears, the value
    * stays the same until a reset command is sent.
    *
    * @see Robot::setCollisionBehavior for setting sensitivity values.
    * @see Robot::automaticErrorRecovery for performing a reset after a collision.
    */
    std::array<double, 6> joint_collision{};

    /**
    * \f$\hat{\tau}_{\text{ext}}\f$
    * Low-pass filtered torques generated by external forces on the joints. It does not include
    * configured end-effector and load nor the mass and dynamics of the robot. tau_ext_hat_filtered
    * is the error between tau_J and the expected torques given by the robot model. Unit: \f$[Nm]\f$.
    */
    std::array<double, 6> tau_ext_hat_filtered{};
    RobotMode robot_mode = RobotMode::kUserStopped;
};

}; // namespace za