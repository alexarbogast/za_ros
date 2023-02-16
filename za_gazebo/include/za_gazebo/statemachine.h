# pragma once

#include <za_hw/za_state.h>
#include <za_gazebo/joint.h>

#include <ros/ros.h>
#include <boost_sml/sml.hpp>

namespace za_gazebo {

using JointMap = std::map<std::string, std::shared_ptr<Joint>>;

// States
struct Idle {};
struct Move {};
struct UserStopped {};

// Events
struct SwitchControl {};
struct UserStop {
    bool pressed;
};

// Gaurds (must be true for event to cause transition)
auto isPressed = [](const UserStop& event) { return event.pressed; };
auto isReleased = [](const UserStop& event, const JointMap& joints) { return not event.pressed; };
auto isStarting = [](const SwitchControl& event, const JointMap& joints) {
    for (auto& joint : joints) {
        if (joint.second->control_method) {
            return true;
        }
    }
    return false;
};
auto isStopping = [](const SwitchControl& event, const JointMap& joints) {
    return not isStarting(event, joints);
};

// Actions
auto start = [](za::RobotState& state) { state.robot_mode = za::RobotMode::kMove; };
auto idle = [](za::RobotState& state) { state.robot_mode = za::RobotMode::kIdle; };
auto stop = [](za::RobotState& state, JointMap& joints) {
    ROS_WARN("User stop pressed, stopping robot");
    state.robot_mode = za::RobotMode::kUserStopped;
    state.q_d = state.q;
    state.dq_d = {0};
    state.ddq_d = {0};
};

struct StateMachine {
    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            // clang-format off
           *state<Idle>        + event<SwitchControl>[isStarting] / start = state<Move>,
            state<Idle>        + event<UserStop>[isPressed]       / stop  = state<UserStopped>,
            //state<Idle>        + event<ErrorRecovery>             / start = state<Move>,
            state<Move>        + event<SwitchControl>[isStopping] / idle  = state<Idle>,
            state<Move>        + event<UserStop>[isPressed]       / stop  = state<UserStopped>,
            state<UserStopped> + event<UserStop>[isReleased]      / idle  = state<Idle>
            // clang-format on
        );
    }
};

}