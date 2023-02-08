#pragma once

#include <array>
#include <string>

#include <za_hw/za_state.h>
#include <za_hw/model_base.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace za_hw {

/**
 * Handle to perform calculations using the dynamic model of a robot.
 */
class ZaModelHandle {
 public:
  ZaModelHandle() = delete;

  /**
   * Creates an instance of a FrankaModelHandle.
   * @param[in] name The name of the model handle.
   * @param[in] model A reference to an implementation of the abstract franka_hw::ModelBase
   * class wrapped by this handle
   * @param[in] robot_state A reference to the current robot state.
   */
  ZaModelHandle(const std::string& name,
                za_hw::ModelBase& model,
                za::RobotState& robot_state)
      : name_(name), model_(&model), robot_state_(&robot_state) {}

  /**
   * Gets the name of the model handle.
   *
   * @return Name of the model handle.
   */
  std::string getName() const noexcept { return name_; }

  /**
   * Calculates the 7x7 mass matrix from the current robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @see franka::Model::mass
   */
  std::array<double, 36> getMass() const { return model_->mass(*robot_state_); }

  /**
   * Calculates the 7x7 mass matrix from a given robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] total_inertia Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @see franka::Model::mass
   */
  std::array<double, 36> getMass(
      const std::array<double, 6>& q,
      const std::array<double, 9>& total_inertia,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal) const {  // NOLINT (readability-identifier-naming)
    return model_->mass(q, total_inertia, total_mass, F_x_Ctotal);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation) from the current robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @return Coriolis force vector.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 6> getCoriolis() const { return model_->coriolis(*robot_state_); }

  /**
   * Calculates the Coriolis force vector (state-space equation) from the given robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] dq Joint velocity. Unit: \f$[\frac{rad}{s}]\f$.
   * @param[in] total_inertia Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 6> getCoriolis(
      const std::array<double, 6>& q,
      const std::array<double, 6>& dq,
      const std::array<double, 9>& total_inertia,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal) const {  // NOLINT (readability-identifier-naming)
    return model_->coriolis(q, dq, total_inertia, total_mass, F_x_Ctotal);
  }

  /**
   * Calculates the gravity vector from the current robot state. Unit: \f$[Nm]\f$.
   *
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   *
   * @see franka::Model::gravity
   */
  std::array<double, 6> getGravity(const std::array<double, 3>& gravity_earth = {
                                       {0., 0., -9.81}}) const {
    return model_->gravity(*robot_state_, gravity_earth);
  }

  /**
   * Calculates the gravity vector from the given robot state. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   *
   * @see franka::Model::gravity
   */
  std::array<double, 6> getGravity(
      const std::array<double, 6>& q,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const {
    return model_->gravity(q, total_mass, F_x_Ctotal, gravity_earth);
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @see franka::Model::pose
   */
  std::array<double, 16> getPose(const za::Frame& frame) const {
    return model_->pose(frame, *robot_state_);
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the given
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @see franka::Model::pose
   */
  std::array<double, 16> getPose(
      const za::Frame& frame,
      const std::array<double, 6>& q,
      const std::array<double, 16>& F_T_EE)  // NOLINT (readability-identifier-naming)
      const {
    return model_->pose(frame, q, F_T_EE);
  }

  /**
   * Gets the 6x6 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x6 matrix in column-major format and calculated from
   * the current robot state.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x6 Jacobian, column-major.
   *
   * @see franka::Model::bodyJacobian
   */
  std::array<double, 36> getBodyJacobian(const za::Frame& frame) const {
    return model_->bodyJacobian(frame, *robot_state_);
  }

  /**
   * Gets the 6x6 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x6 matrix in column-major format and calculated from
   * the given robot state.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x6 Jacobian, column-major.
   *
   * @see franka::Model::bodyJacobian
   */
  std::array<double, 36> getBodyJacobian(
      const za::Frame& frame,
      const std::array<double, 6>& q,
      const std::array<double, 16>& F_T_EE)  // NOLINT (readability-identifier-naming)
      const {
    return model_->bodyJacobian(frame, q, F_T_EE);
  }

  /**
   * Gets the 6x6 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x6 matrix in column-major format and calculated from
   * the current robot state.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x6 Jacobian, column-major.
   *
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 36> getZeroJacobian(const za::Frame& frame) const {
    return model_->zeroJacobian(frame, *robot_state_);
  }

  /**
   * Gets the 6x6 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x6 matrix in column-major format and calculated from
   * the given robot state.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x6 Jacobian, column-major.
   *
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 36> getZeroJacobian(
      const za::Frame& frame,
      const std::array<double, 6>& q,
      const std::array<double, 16>& F_T_EE)  // NOLINT (readability-identifier-naming)
      const {
    return model_->zeroJacobian(frame, q, F_T_EE);
  }

 private:
  std::string name_;
  const za_hw::ModelBase* model_;
  const za::RobotState* robot_state_;
};

/**
 * Hardware interface to perform calculations using the dynamic model of a robot.
 */
class ZaModelInterface : public hardware_interface::HardwareResourceManager<ZaModelHandle> {
};

}  // namespace franka_hw