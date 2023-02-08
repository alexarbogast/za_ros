#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace hydra_control {

class JointVelocityController : public controller_interface::MultiInterfaceController<
                                    hardware_interface::VelocityJointInterface,
                                    hydra_hw::HydraStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void starting(const ros::Time&) override;
        void stopping(const ros::Time&) override;

    private:
        hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
        ros::Duration elapsed_time_;
};

} // namespace hydra_control