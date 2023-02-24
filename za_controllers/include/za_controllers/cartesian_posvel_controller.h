#pragma once

#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <ros/node_handle.h>
#include <za_msgs/PosVelSetpoint.h>
#include <Eigen/Dense>

namespace za_controllers {

class CartesianPosVelController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           za_hw::ZaModelInterface,
                                           za_hw::ZaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

private:
    //hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::unique_ptr<za_hw::ZaStateHandle> state_handle_;
    std::unique_ptr<za_hw::ZaModelHandle> model_handle_;

    // setpoint
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Matrix<double, 6, 1> twist_setpoint_;
    std::mutex pose_twist_setpoint_mutex_;

    double Kp, Ko;

    ros::Subscriber sub_command_;
    void commandCallback(const za_msgs::PosVelSetpointPtr& msg);
};

} // namespace za_controllers