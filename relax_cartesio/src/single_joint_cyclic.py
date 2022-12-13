#!/usr/bin/env python

from xbot_interface import xbot_interface as xbi
from xbot_interface import config_options as co

import rospy
import numpy as np
import math 
from os.path import join, dirname, abspath


# some utils
def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', False)
    return cfg

# get model, robot
prefix = 'xbotcore'
model_cfg = get_xbot_cfg(rospy.get_param(prefix + '/robot_description'),
                         rospy.get_param(prefix + '/robot_description_semantic'))

# get robot from standard urdf
robot = xbi.RobotInterface(model_cfg)
robot.sense()

# set control mode
robot.setControlMode(xbi.ControlMode.Idle())

# joint to move
jname = 'relax_arm1_joint3'
ji = robot.getDofIndex(jname)
robot.setControlMode(
    {
        jname: xbi.ControlMode.Position()
    }
)

# start pos
q0 = robot.getPositionReference()[ji]

# trj parameters

A = 0.5
total_time = 60
freq = 0.025
freq = 0.30
# freq = 1.00

rospy.init_node('single_joint_cyclic_node')

period = 1/freq
n_iter = int(total_time/period)
i = 0
time = 0
dt = 0.005
rate = rospy.Rate(1/dt)

print('started!')
while not rospy.is_shutdown() and time < n_iter*period:
    qref = A*math.sin(2*math.pi*freq*time)
    robot.setPositionReference({jname: qref})
    robot.move()
    time += dt
    rate.sleep()

print('done')