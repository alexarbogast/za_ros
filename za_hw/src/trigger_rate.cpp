// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <za_hw/trigger_rate.h>

namespace robot_hw {

TriggerRate::TriggerRate(double rate) : period_(1.0 / rate) 
{
    this->clock_ = rclcpp::Clock::make_shared();
    this->time_stamp_ = this->clock_->now();
}

bool TriggerRate::operator()() {
    if ((this->clock_->now() - time_stamp_).seconds() > period_) {
        time_stamp_ = this->clock_->now();
        return true;
    }
    return false;
}

}  // namespace robot_hw