#include <controller_interface/multi_interface_controller.h>
#include <za_hw/za_state_interface.h>
#include <za_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <za_msgs/ZaState.h>

namespace za_control
{

class ZaStateController
    : public controller_interface::MultiInterfaceController<za_hw::ZaStateInterface> 
{
public:
    ZaStateController() = default;

    /**
    * Initializes the controller with interfaces and publishers.
    *
    * @param[in] robot_hardware RobotHW instance to get a franka_hw::FrankaStateInterface from.
    * @param[in] root_node_handle Node handle in the controller_manager namespace.
    * @param[in] controller_node_handle Node handle in the controller namespace.
    */
    bool init(hardware_interface::RobotHW* robot_hardware,
              ros::NodeHandle& root_node_handle,
              ros::NodeHandle& controller_node_handle) override;

    /**
   * Reads the current robot state from the franka_hw::FrankaStateInterface and publishes it.
   *
   * @param[in] time Current ROS time.
   * @param[in] period Time since the last update.
   */
   void update(const ros::Time& time, const ros::Duration& period) override;

private:
    void publishJointStates(const ros::Time& time);
    void publishZaStates(const ros::Time& time);

    std::string arm_id_;

    za_hw::ZaStateInterface* za_state_interface_{};
    std::unique_ptr<za_hw::ZaStateHandle> za_state_handle_{};

    realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_desired_;
    realtime_tools::RealtimePublisher<za_msgs::ZaState> publisher_za_states_;

    robot_hw::TriggerRate trigger_publish_;
    za::RobotState robot_state_;
    uint64_t sequence_number_ = 0;
    std::vector<std::string> joint_names_;
};

} // namespace za_control