// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <za_hw/trigger_rate.h>

namespace robot_hw {

TriggerRate::TriggerRate(double rate) : period_(1.0 / rate), time_stamp_(ros::Time::now()) {}

bool TriggerRate::operator()() {
  if ((ros::Time::now() - time_stamp_).toSec() > period_) {
    time_stamp_ = ros::Time::now();
    return true;
  }
  return false;
}

}  // namespace robot_hw