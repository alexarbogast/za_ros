#!/usr/bin/env python

import os
import rospy
from za_hal_plumber import HALPlumberSim, HALPlumberEC

#####################################
# Sim mode or real hardware?
sim_mode = rospy.get_param("sim_mode", False)
# Path for halfiles
halpath = rospy.get_param("hal_mgr/hal_file_dir")
# For locating sim_startup_selector
os.environ['PATH'] = '{}:{}'.format(os.environ['PATH'], halpath)

# read joint configuration from ROS parameters
jconfig_raw = rospy.get_param("hardware_settings")

# Turn params into dict of dicts
joint_config = dict()
for k, v in jconfig_raw.items():
    variables_list = v
    var_dict = dict()
    for var_item in variables_list:
        # each item is a dict with a single key, and we want one single dict
        for key, value in var_item.items():
            var_dict[key] = value
    joint_config[k] = var_dict

# Generate configuration
config = dict(
    # Above per-joint configuration
    joint_config=joint_config,
    # Name of cpuset cgroup
    cgname=rospy.get_param("hal_mgr/rt_cgname", None),
    # Name of HAL thread
    thread_name='robot_hw_thread',
    # Period of the HAL thread
    thread_period=1000000,
    # LCEC config file
    lcec_config_file=os.path.join(
        halpath, rospy.get_param("hal_mgr/lcec_config_file")
    ),
)

# Select config type
if not sim_mode:
    # EtherCAT
    hal_plumber_class = HALPlumberEC
else:
    # Simulation
    hal_plumber_class = HALPlumberSim

# Set up HAL
rospy.loginfo("Starting HAL in %s mode" % hal_plumber_class.mode_name)
hp = hal_plumber_class(config)
hp.setup_hal()
