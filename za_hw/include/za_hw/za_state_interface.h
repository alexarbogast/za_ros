#pragma once

#include <za_hw/za_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace za_hw
{
class ZaStateHandle {
public:
    ZaStateHandle() = delete;

    /**
    * Creates an instance of a ZaStateHandle.
    *
    * @param[in] name The name of the state handle.
    * @param[in] robot_state A reference to the robot state wrapped by this handle.
    */
    ZaStateHandle(const std::string& name, za::RobotState& robot_state)
      : name_(name), robot_state_(&robot_state) {}

    /**
    * Gets the name of the state handle.
    *
    * @return Name of the state handle.
    */
    const std::string& getName() const noexcept { return name_; }

    /**
    * Gets the current robot state.
    *
    * @return Current robot state.
    */
    const za::RobotState& getRobotState() const noexcept { return *robot_state_; }

private:
    std::string name_;
    const za::RobotState* robot_state_;
};

/**
* Hardware interface to read the complete robot state.
*
* @see za::RobotState for a description of the values included in the robot state.
*/
class ZaStateInterface : public hardware_interface::HardwareResourceManager<ZaStateHandle> {
};

} // namespace za_hw