#include <za_control/za_state_controller.h>
#include <pluginlib/class_list_macros.h>

namespace za_control
{

bool ZaStateController::init(hardware_interface::RobotHW* robot_hardware,
                             ros::NodeHandle& root_node_handle,
                             ros::NodeHandle& controller_node_handle)
{
    za_state_interface_ = robot_hardware->get<za_hw::ZaStateInterface>();
    if (za_state_interface_ == nullptr) {
        ROS_ERROR("ZaStateController: Could not get Za state interface from hardware");
        return false;
    }
    if (!controller_node_handle.getParam("arm_id", arm_id_)) {
        ROS_ERROR("ZaStateController: Could not get parameter arm_id");
        return false;
    }
    double publish_rate(30.0);
    if (!controller_node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("ZaStateController: Did not find publish_rate. Using default "
                         << publish_rate << " [Hz].");
    }

    if (!controller_node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != robot_state_.q.size()) {
    ROS_ERROR(
        "ZaStateController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
    }

    try {
        za_state_handle_ = std::make_unique<za_hw::ZaStateHandle>(
            za_state_interface_->getHandle(arm_id_ + "_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("ZaStateController: Exception getting za state handle: " << ex.what());
        return false;
    }

    publisher_joint_states_.init(controller_node_handle, "joint_states", 1);
    publisher_joint_states_desired_.init(controller_node_handle, "joint_states_desired", 1);
    publisher_za_states_.init(controller_node_handle, "za_states", 1);

    {
        std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> lock(
            publisher_joint_states_);
        publisher_joint_states_.msg_.name.resize(joint_names_.size());
        publisher_joint_states_.msg_.position.resize(robot_state_.q.size());
        publisher_joint_states_.msg_.velocity.resize(robot_state_.dq.size());
    }
    {
        std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> lock(
            publisher_joint_states_desired_);
        publisher_joint_states_desired_.msg_.name.resize(joint_names_.size());
        publisher_joint_states_desired_.msg_.position.resize(robot_state_.q_d.size());
        publisher_joint_states_desired_.msg_.velocity.resize(robot_state_.dq_d.size());
    }
    return true;
}

void ZaStateController::update(const ros::Time& time, const ros::Duration& period)
{
    if (trigger_publish_())
    {
        robot_state_ = za_state_handle_->getRobotState();
        publishJointStates(time);
        publishZaStates(time);
        sequence_number_++;
    }
}

void ZaStateController::publishJointStates(const ros::Time& time)
{
    if (publisher_joint_states_.trylock()) 
    {
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                  "Robot state joint members do not have same size");
        for (size_t i = 0; i < robot_state_.q.size(); i++) 
        {
            publisher_joint_states_.msg_.name[i] = joint_names_[i];
            publisher_joint_states_.msg_.position[i] = robot_state_.q[i];
            publisher_joint_states_.msg_.velocity[i] = robot_state_.dq[i];
        }
        publisher_joint_states_.msg_.header.stamp = time;
        publisher_joint_states_.msg_.header.seq = sequence_number_;
        publisher_joint_states_.unlockAndPublish();
    }
    if (publisher_joint_states_desired_.trylock())
    {
        static_assert(sizeof(robot_state_.q_d) == sizeof(robot_state_.dq_d),
                      "Robot state joint members do not have same size");
        for (size_t i = 0; i < robot_state_.q_d.size(); i++)
        {
            publisher_joint_states_desired_.msg_.name[i] = joint_names_[i];
            publisher_joint_states_desired_.msg_.position[i] = robot_state_.q_d[i];
            publisher_joint_states_desired_.msg_.velocity[i] = robot_state_.dq_d[i];
        }
        publisher_joint_states_desired_.msg_.header.stamp = time;
        publisher_joint_states_desired_.msg_.header.seq = sequence_number_;
        publisher_joint_states_desired_.unlockAndPublish();
    }
}

void ZaStateController::publishZaStates(const ros::Time& time) {
    if (publisher_za_states_.trylock()) {
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.q_d),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq_d),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.ddq_d),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.theta),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dtheta),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_collision),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_contact),
                    "Robot state joint members do not have same size");
        static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_ext_hat_filtered),
                    "Robot state joint members do not have same size");
        for (size_t i = 0; i < robot_state_.q.size(); i++) {
        publisher_za_states_.msg_.q[i] = robot_state_.q[i];
        publisher_za_states_.msg_.q_d[i] = robot_state_.q_d[i];
        publisher_za_states_.msg_.dq[i] = robot_state_.dq[i];
        publisher_za_states_.msg_.dq_d[i] = robot_state_.dq_d[i];
        publisher_za_states_.msg_.ddq_d[i] = robot_state_.ddq_d[i];
        publisher_za_states_.msg_.theta[i] = robot_state_.theta[i];
        publisher_za_states_.msg_.dtheta[i] = robot_state_.dtheta[i];
        publisher_za_states_.msg_.joint_collision[i] = robot_state_.joint_collision[i];
        publisher_za_states_.msg_.joint_contact[i] = robot_state_.joint_contact[i];
        publisher_za_states_.msg_.tau_ext_hat_filtered[i] = robot_state_.tau_ext_hat_filtered[i];
        }

        static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.F_T_EE),
                    "Robot state transforms do not have same size");
        static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.F_T_NE),
                    "Robot state transforms do not have same size");
        static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.NE_T_EE),
                    "Robot state transforms do not have same size");
        static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.O_T_EE_d),
                    "Robot state transforms do not have same size");
        for (size_t i = 0; i < robot_state_.O_T_EE.size(); i++) {
            publisher_za_states_.msg_.O_T_EE[i] = robot_state_.O_T_EE[i];
            publisher_za_states_.msg_.F_T_EE[i] = robot_state_.F_T_EE[i];
            publisher_za_states_.msg_.F_T_NE[i] = robot_state_.F_T_NE[i];
            publisher_za_states_.msg_.NE_T_EE[i] = robot_state_.NE_T_EE[i];
            publisher_za_states_.msg_.O_T_EE_d[i] = robot_state_.O_T_EE_d[i];
        }
        publisher_za_states_.msg_.m_ee = robot_state_.m_ee;
        publisher_za_states_.msg_.m_load = robot_state_.m_load;
        publisher_za_states_.msg_.m_total = robot_state_.m_total;

        for (size_t i = 0; i < robot_state_.I_load.size(); i++) {
            publisher_za_states_.msg_.I_ee[i] = robot_state_.I_ee[i];
            publisher_za_states_.msg_.I_load[i] = robot_state_.I_load[i];
            publisher_za_states_.msg_.I_total[i] = robot_state_.I_total[i];
        }

        for (size_t i = 0; i < robot_state_.F_x_Cload.size(); i++) {
            publisher_za_states_.msg_.F_x_Cee[i] = robot_state_.F_x_Cee[i];
            publisher_za_states_.msg_.F_x_Cload[i] = robot_state_.F_x_Cload[i];
            publisher_za_states_.msg_.F_x_Ctotal[i] = robot_state_.F_x_Ctotal[i];
        }

        publisher_za_states_.msg_.header.seq = sequence_number_;
        publisher_za_states_.msg_.header.stamp = time;
        publisher_za_states_.unlockAndPublish();
    }
}

} // namespace za_control

PLUGINLIB_EXPORT_CLASS(za_control::ZaStateController, controller_interface::ControllerBase)