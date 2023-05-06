#include <za_controllers/task_priority_trajectory_controller.h>

#include <za_controllers/pseudo_inversion.h>
#include <pluginlib/class_list_macros.h>


namespace za_controllers {

const static std::string param_name = "joints"; 

bool TaskPriorityTrajectoryController::init(hardware_interface::RobotHW* robot_hw,
                                        ros::NodeHandle& node_handle) {
    // read rosparams
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("TaskPriorityController: Could not get parameter arm_id");
        return false;
    }

    if (not node_handle.getParam("Kp", Kp_) or 
        not node_handle.getParam("Ko", Ko_) or
        not node_handle.getParam("Kr", Kr_)) {
        ROS_ERROR("Missing controller gains 'Kp' or 'Ko' or 'Kr'");
        return false;
    }

    std::vector<double> z_align;
    if (not node_handle.getParam("z_align", z_align)) {
        ROS_ERROR("Missing z alignment axis 'z_align'");
        return false;
    }
    z_align_ = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>
        (z_align.data(), z_align.size());

    auto* velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
            "TaskPriorityTrajectoryController: Could not get Cartesian velocity interface from "
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
        ROS_ERROR_STREAM("TaskPriorityTrajectoryController: Wrong number of joint names, got "
            << joint_names.size() << ". Expected 6");
    }
    joint_handles_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        try {
            joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "TaskPriorityTrajectoryController: Exception getting joint handles: " << e.what());
            return false;
        }
    }
    
    auto* model_interface = robot_hw->get<za_hw::ZaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("TaskPriorityTrajectoryController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<za_hw::ZaModelHandle>(
                            model_interface->getHandle(arm_id + "_model"));
                        
    } catch (hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "TaskPriorityTrajectoryController: Exception getting model handle from interface: " << e.what());
        return false;
    }

    auto* state_interface = robot_hw->get<za_hw::ZaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("TaskPriorityTrajectoryController: Could not get za state interface from hardware");
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
                    "TaskPriorityTrajectoryController: Robot is not in the expected starting position ");
                return false;
            }
        }

    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "TaskPriorityTrajectoryController: Exception getting state handle: " << e.what());
        return false;
    }

    dynamic_reconfigure_posvel_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_taskpriority_param_node");
    
    dynamic_server_posvel_param_ = std::make_unique<
        dynamic_reconfigure::Server<za_controllers::taskpriority_paramConfig>>(
            dynamic_reconfigure_posvel_param_node_);
    
    dynamic_server_posvel_param_->setCallback(
        boost::bind(&TaskPriorityTrajectoryController::taskpriorityParamCallback, this, _1, _2));
    
    publisher_command_.init(node_handle, "cart_command", 1);
    TrajectoryAdapter::init(node_handle);
    return true;
}

void TaskPriorityTrajectoryController::starting(const ros::Time&) {
    za::RobotState initial_state = state_handle_->getRobotState();
    Eigen::Affine3d initial_transformation(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    setpoint_.p = initial_transformation.translation();
    setpoint_.q = Eigen::Quaterniond(initial_transformation.rotation());
}

void TaskPriorityTrajectoryController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    // ================== trajectory generation ===================
    if (action_server_->isActive() && !done_.load()) {
        std::lock_guard<std::mutex> lock_trajectory(gen_->traj_lock_);
        done_ = gen_->sample(period.toSec(), setpoint_);
    }

    // ================== inverse jacobian control ================
    za::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Affine3d pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

    std::array<double, 36> jacobian_array = 
        model_handle_->getZeroJacobian(za::Frame::kEndEffector);    
    Eigen::Map<Eigen::Matrix<double, 6, 6>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_pinv;
    za_controllers::pseudoInverse(jacobian, jacobian_pinv, false);

    /* task tracking */ 
    Eigen::Vector3d pose_error = setpoint_.p - pose.translation();

    const auto& z_eef = pose.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d orient_error = z_eef.cross(z_align_);

    Eigen::Matrix<double, 6, 1> error(6);
    error << Kp_ * pose_error, Ko_ * orient_error;

    Eigen::Matrix<double, 6, 1> dp_d;
    dp_d << setpoint_.v, setpoint_.w;
    dp_d += error;

    /* redundancy resolution */
    Eigen::MatrixXd null_project = Eigen::Matrix<double, 6, 6>::Identity() 
        - jacobian_pinv * jacobian;
    Eigen::Matrix<double, 6, 1> dp_redundancy = Eigen::Matrix<double, 6, 1>::Zero();

    std::array<double, 216> hessian_array =
        model_handle_->getZeroHessian(za::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 36, 6>> hessian(hessian_array.data());

    Eigen::Matrix<double, 6, 6> J_Jt = jacobian * jacobian.transpose();
    double det_J_Jt = J_Jt.determinant();
    auto inv_J_Jt = J_Jt.inverse();

    Eigen::Matrix<double, 36, 1> vec_inv_J_Jt;
    Eigen::MatrixXd::Map(&vec_inv_J_Jt[0], 6, 6) = inv_J_Jt;
    
    // find manipulability Jacobian
    Eigen::Matrix<double, 6, 1> Jm;
    Jm.setZero();
    for (int i = 0; i < 6; i++) {
        const auto& Hi = hessian.block<6, 6>(i * 6, 0);
        Eigen::Matrix<double, 36, 1> vec_J_HiT;
        Eigen::MatrixXd::Map(&vec_J_HiT[0], 6, 6) = jacobian * Hi.transpose();

        Jm(i, 0) = sqrt(abs(det_J_Jt)) * vec_J_HiT.dot(vec_inv_J_Jt);
    }

    // use redundant axis (z-rotation) to drive the posture to maximum manipulability
    dp_redundancy = -Kr_ * jacobian * Jm;
    dp_redundancy.block<5, 1>(0, 0).setZero();

    Eigen::Matrix<double, 6, 1> dq_cmd = (jacobian_pinv * (dp_d + dp_redundancy));

    for (size_t i = 0; i < 6; ++i) {
        joint_handles_[i].setCommand(dq_cmd(i));
    }
} 

void TaskPriorityTrajectoryController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void TaskPriorityTrajectoryController::taskpriorityParamCallback(za_controllers::taskpriority_paramConfig& config,
                                                                 uint32_t /*level*/) {
    Kp_ = config.translation_gain;
    Ko_ = config.rotation_gain;
    Kr_ = config.redundancy_gain;
}

} // namespace za_controllers

PLUGINLIB_EXPORT_CLASS(za_controllers::TaskPriorityTrajectoryController, controller_interface::ControllerBase)