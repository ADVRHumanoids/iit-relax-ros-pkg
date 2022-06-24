#!/usr/bin/python3

import numpy as np
import time
import rospy
from std_msgs.msg import Float64
import rospkg
import yaml
from os.path import join, dirname, abspath
import os 
import argparse
from math import cos, sin, trunc

os.environ['XBOT_VERBOSE'] = '2'
from xbot_interface import xbot_interface as xbi 
from xbot_interface import config_options as co 
from cartesian_interface.pyci_all import *

# some utility functions
def get_relax_urdf_srdf():
    rospack = rospkg.RosPack()
    relax_urdf_path = join(rospack.get_path('relax_urdf'), 'urdf/relax_arm.urdf')
    relax_srdf_path = join(rospack.get_path('relax_srdf'), 'srdf/relax_arm.srdf')
    paths = [relax_urdf_path, relax_srdf_path] 
    return (open(p, 'r').read() for p in paths) 

def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', False)
    return cfg

def quintic(alpha):
    if alpha < 0:
        return 0
    elif alpha > 1:
        return 1
    else:
        return ((6*alpha - 15)*alpha + 10)*alpha**3


# initialize ros
rospy.init_node('relax_interaction_driving')

# get augmented kinematic model for ik
model_cfg = get_xbot_cfg(*get_relax_urdf_srdf())
model = xbi.ModelInterface(model_cfg)

# get robot from standard urdf
ctrl_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'), 
                        rospy.get_param('/xbotcore/robot_description_semantic')) 
robot = xbi.RobotInterface(ctrl_cfg)
robot.sense()

# set control mode

# whole robot
robot.setControlMode(xbi.ControlMode.PosImpedance())

# set model state a
qref = robot.eigenToMap(robot.getPositionReference())
model.setJointPosition(qref)
model.update()


# set lower impedance for "right arm interface"
robot.setStiffness({"relax_arm1_joint0": 100})
robot.setStiffness({"relax_arm1_joint1": 100})
robot.move()

# safe sleep
time.sleep(2)

#starting pose
robot.sense()
zero_pose = robot.model().getPose('relax_arm1_linkEndEffector')
print(zero_pose)

# prepare ROS stuffs
pub_cart_error_x = rospy.Publisher('cart_error_x', Float64, queue_size=10)
pub_cart_error_y = rospy.Publisher('cart_error_y', Float64, queue_size=10)

car_vel_ref_x = rospy.Publisher('car_vel_ref_x', Float64, queue_size=10)
car_vel_ref_teta = rospy.Publisher('car_vel_ref_teta', Float64, queue_size=10)

print('started looping..')
time_t = 0.0
dt = 0.01
rate = rospy.Rate(1./dt * 1.0)

while not rospy.is_shutdown():

    car_vel_ref = np.zeros((6))

    robot.sense()
    cart_error_x = zero_pose.translation[0] - robot.model().getPose('relax_arm1_linkEndEffector').translation[0]
    cart_error_y = zero_pose.translation[1] - robot.model().getPose('relax_arm1_linkEndEffector').translation[1]

    pub_cart_error_x.publish(cart_error_x)
    pub_cart_error_y.publish(cart_error_y)

    k = 3
    max_vel_x = 0.25
    max_vel_teta = 0.2

    car_vel_ref[0:3] = -(k * cart_error_x)
    car_vel_ref[5] = -(k * cart_error_y)

    # saturation
    if abs(car_vel_ref[0]) > max_vel_x:
        car_vel_ref[0] = np.sign(car_vel_ref[0]) * max_vel_x

    if abs(car_vel_ref[5]) > max_vel_teta:
        car_vel_ref[5] = np.sign(car_vel_ref[5]) * max_vel_teta

    car_vel_ref_x.publish(car_vel_ref[0])
    car_vel_ref_teta.publish(car_vel_ref[5])

    # send vel ref to base

    # sleep
    rate.sleep()
    time_t += dt
    