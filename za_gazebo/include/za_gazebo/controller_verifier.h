#pragma once
#include <za_gazebo/joint.h>
#include <hardware_interface/interface_resources.h>
#include <hardware_interface/controller_info.h>
#include <boost/optional.hpp>

namespace za_gazebo 
{
class ControllerVerifier
{
public:
    /// Creates a ControllerVerifier object to check controllers for franka_gazebo
    /// @param joints map of joint names and joints
    /// @param arm_id prefix of the joints
    ControllerVerifier(const std::map<std::string, std::shared_ptr<za_gazebo::Joint>>& joints,
                       const std::string& arm_id);

    bool isValidController(const hardware_interface::ControllerInfo& controller) const;
    bool areValidJoints(const std::set<std::string>& resources) const;

    static boost::optional<ControlMethod> determineControlMethod(
        const std::string& hardware_interface);
    
    private:
        std::vector<std::string> joint_names_;
        std::string arm_id_;

        static bool hasControlMethod(const hardware_interface::InterfaceResources& resource);
};

} // namespace za_gazebo