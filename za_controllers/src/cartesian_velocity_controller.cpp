#include <za_controllers/cartesian_velocity_controller.h>
#include <za_controllers/pseudo_inversion.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace za_controllers {

const static std::string param_name = "joints"; 

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hw,
                                       ros::NodeHandle& node_handle) {
    sub_command_ = node_handle.subscribe(
        "command", 1, &CartesianVelocityController::commandCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());
    
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
        return false;
    }

    auto* velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
            "CartesianVelocityController: Could not get Cartesian velocity interface from "
            "hardware");
        return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam(param_name, joint_names)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << 
            "'(namespace: " << node_handle.getNamespace() << ")");
        return false; 
    }
    if (joint_names.size() != 6) {
        ROS_ERROR_STREAM("CarteisanVelocityController: Wrong number of joint names, got "
            << joint_names.size() << ". Expected 6");
    }
    joint_handles_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        try {
            joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "CartesianVelocityController: Exception getting joint handles: " << e.what());
            return false;
        }
    }
    
    auto* model_interface = robot_hw->get<za_hw::ZaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("CarteisanVelocityController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<za_hw::ZaModelHandle>(
                            model_interface->getHandle(arm_id + "_model"));
                        
    } catch (hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "CarteisanVelocityController: Exception getting model handle from interface: " << e.what());
        return false;
    }

    auto* state_interface = robot_hw->get<za_hw::ZaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("CartesianVelocityController: Could not get za state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<za_hw::ZaStateHandle>(
                            state_interface->getHandle(arm_id + "_robot"));

        // we must start in a known (non-singular) position
        // possibly pass this as a parameter
        std::array<double, 6> q_start = {{0, 0.53, 0.47, 0, -1, 0}};
        for (size_t i = 0; i < q_start.size(); i++) {
            if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
                ROS_ERROR_STREAM(
                    "CartesianVelocityController: Robot is not in the expected starting position ");
                return false;
            }
        }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "CartesianVelocityController: Exception getting state handle: " << e.what());
        return false;
    }

    twist_setpoint_.setZero();

    return true;
}

void CartesianVelocityController::starting(const ros::Time&) {

}

void CartesianVelocityController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    za::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 36> jacobian_array = 
        model_handle_->getZeroJacobian(za::Frame::kEndEffector);
        
    Eigen::Map<Eigen::Matrix<double, 6, 6>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_pinv;
    za_controllers::pseudoInverse(jacobian, jacobian_pinv, false);

    Eigen::Matrix<double, 6, 1> dp_d(this->twist_setpoint_);
    Eigen::Matrix<double, 6, 1> dq_cmd;
    dq_cmd = jacobian_pinv * dp_d;

    for (size_t i = 0; i < 6; ++i) {
        joint_handles_[i].setCommand(dq_cmd(i));
    }
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityController::commandCallback(
        const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> twist_setpoint_mutex_lock(twist_setpoint_mutex_);
    twist_setpoint_ << msg->linear.x, msg->linear.y, msg->linear.z,
                       msg->angular.x, msg->angular.y, msg->angular.z;
}

} // namespace za_controllers

PLUGINLIB_EXPORT_CLASS(za_controllers::CartesianVelocityController, controller_interface::ControllerBase)