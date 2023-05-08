#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/za_model_interface.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <cartesian_trajectory_adapter/cartesian_trajectory_adapter.h>

#include <za_controllers/cartesian_trajectory_paramConfig.h>


namespace za_controllers {
using TrajectoryAdapter = cartesian_trajectory_controllers::CartesianTrajectoryAdapter; 

class CartesianTrajectoryController : public controller_interface::MultiInterfaceController<
                                             hardware_interface::VelocityJointInterface,
                                             za_hw::ZaModelInterface,
                                             za_hw::ZaStateInterface> 
                                    , public TrajectoryAdapter {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

    void adapterStateCallback(cartesian_controllers::CartesianState& state) const;
private:   
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::unique_ptr<za_hw::ZaStateHandle> state_handle_;
    std::unique_ptr<za_hw::ZaModelHandle> model_handle_;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<za_controllers::cartesian_trajectory_paramConfig>>
        dynamic_server_posvel_param_;
    ros::NodeHandle dynamic_reconfigure_posvel_param_node_;
    double Kp_, Ko_;
    void posvelParamCallback(za_controllers::cartesian_trajectory_paramConfig& config,
                             uint32_t level);

    cartesian_controllers::CartesianState setpoint_;

    // temporary
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> publisher_command_;
};

} // namespace za_controllers