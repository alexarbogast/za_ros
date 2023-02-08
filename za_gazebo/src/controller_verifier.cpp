#include <za_gazebo/controller_verifier.h>

namespace za_gazebo
{
ControllerVerifier::ControllerVerifier(
        const std::map<std::string, std::shared_ptr<za_gazebo::Joint>>& joints,
        const std::string& arm_id)
    : arm_id_(arm_id) {
    for (const auto& joint : joints) {
        joint_names_.push_back(joint.first);
    }
}

bool ControllerVerifier::isValidController(
    const hardware_interface::ControllerInfo& controller) const {
    for (const auto& claimed_resource : controller.claimed_resources) {
        if (hasControlMethod(claimed_resource)) {
            return true;
        }
    }
    return std::none_of(
        controller.claimed_resources.begin(), controller.claimed_resources.end(),
        [](const auto& resource) {
            return ControllerVerifier::determineControlMethod(resource.hardware_interface);
        });
}

bool ControllerVerifier::hasControlMethod(const hardware_interface::InterfaceResources& resource) {
    return ControllerVerifier::determineControlMethod(resource.hardware_interface).is_initialized();
}

boost::optional<ControlMethod> ControllerVerifier::determineControlMethod(
    const std::string& hardware_interface) {
    if (hardware_interface.find("hardware_interface::PositionJointInterface") != std::string::npos) {
      return POSITION;
    }
    if (hardware_interface.find("hardware_interface::VelocityJointInterface") != std::string::npos) {
      return VELOCITY;
    }
    if (hardware_interface.find("hardware_interface::EffortJointInterface") != std::string::npos) {
      return EFFORT;
    }
    return boost::none;
}

} // namespace za_gazebo