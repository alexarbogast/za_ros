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

    publisher_joint_states_.init(controller_node_handle, "joint_state", 1);
    publisher_joint_states_desired_.init(controller_node_handle, "joint_states_desired", 1);

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

} // namespace za_control

PLUGINLIB_EXPORT_CLASS(za_control::ZaStateController, controller_interface::ControllerBase)