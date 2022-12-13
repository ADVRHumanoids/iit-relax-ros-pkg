#!/usr/bin/env python


from cartesian_interface.pyci_all import *
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

def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt
    model.setJointPosition(q)
    model.update()
    return q, qdot

def get_ikpb():
    ikpath = join(dirname(abspath(__file__)), '..', 'relax_arm_stack.yaml')
    with open(ikpath, 'r') as f:
        return f.read()

def loop_until_stop(ci, model, time, dt):
    rate = rospy.Rate(1/dt)
    while True and not rospy.is_shutdown():

        robot.sense()

        q, qdot = update_ik(ci, model, time, dt)

        rspub.publishTransforms('ci')
        
        # send model state as reference
        qref = model.getJointPositionMap()
        dqref = model.eigenToMap(model.getJointVelocity())

        robot.setPositionReference(qref)
        robot.setVelocityReference(dqref)
        robot.move()

        time += dt

        if np.linalg.norm(robot.getMotorVelocity()) < 1e-3:
            break

        rate.sleep()
    
    return time

def loop_until_reached(ci, model, time, dt, tasks, cb=None):
    rate = rospy.Rate(1/dt)
    
    while not all([t.getTaskState() == pyci.State.Online for t in tasks]) and not rospy.is_shutdown():

        robot.sense()
        
        q, qdot = update_ik(ci, model, time, dt)

        rspub.publishTransforms('ci')
        
        # send model state as reference
        qref = model.getJointPositionMap()
        dqref = model.eigenToMap(model.getJointVelocity())

        robot.setPositionReference(qref)
        robot.setVelocityReference(dqref)
        robot.move()

        if cb is not None:
            cb()

        time += dt

        rate.sleep()

    return time


# get model, robot
prefix = 'xbotcore'
model_cfg = get_xbot_cfg(rospy.get_param(prefix + '/robot_description'),
                         rospy.get_param(prefix + '/robot_description_semantic'))
model = xbi.ModelInterface(model_cfg)

# get robot from standard urdf
robot = xbi.RobotInterface(model_cfg)
robot.sense()

# set control mode
robot.setControlMode(xbi.ControlMode.Position())

# set model state and fix base position
qref = robot.eigenToMap(robot.getRobotState('home'))
model.setJointPosition(qref)
model.update()

# check that we start from home
qerr = robot.getPositionReference() - robot.mapToEigen(qref)
if np.linalg.norm(qerr) > 1e-2:
    print('Error: robot state is not home')
    print(robot.mapToEigen(qref))
    print(robot.getPositionReference())
    exit(1)

# init rospy
rospy.init_node('test_poses')

# time
time = 0
dt = 0.005
rate = rospy.Rate(1/dt)

# ci
ci = pyci.CartesianInterface.MakeInstance('OpenSot', get_ikpb(), model, dt)
rspub = pyci.RobotStatePublisher(model)

# tasks
tcp_name = 'relax_arm1_linkEndEffector'
tcp = ci.getTask('arm_control')

# some parameters
vavg = 0.05
vavg = 0.15
# vavg = 0.3

ytrj = 0.4
time_to_go = 2*ytrj/vavg
print(f'trj will require {time_to_go} seconds')
n_iter = 30

# log function
ee_pos = []
ee_rot = []

ee_pos_des = []
ee_rot_err = []

def log_error():
    tcp_ref = tcp.getPoseReference()[0]
    tcp_pose = model.getPose(tcp_name)
    err = tcp_pose*tcp_tgt.inverse()
    ee_pos.append(tcp_pose.translation)
    ee_rot.append(tcp_pose.quaternion)
    ee_pos_des.append(tcp_ref.translation)
    ee_rot_err.append(err.quaternion)

# motion
i = 0
tcp_tgt = tcp.getPoseReference()[0]
goal_pos = []
goal_pos_des = []
while not rospy.is_shutdown() and i < n_iter:

    print(f'iteration {i+1}/{n_iter}')
 
    tcp_tgt.translation = [0.3, 0.4, 0.9]
    tcp.setPoseTarget(tcp_tgt, time_to_go)
    time = loop_until_reached(ci, model, time, dt, [tcp], cb=log_error)
    time = loop_until_stop(ci, model, time, dt)

    robot.sense()
    tcp_pose = robot.model().getPose(tcp_name)
    goal_pos.append(tcp_pose.translation)
    goal_pos_des.append(tcp_tgt.translation)

    tcp_tgt.translation = [0.3, -0.4, 0.9]
    tcp.setPoseTarget(tcp_tgt, time_to_go)
    time = loop_until_reached(ci, model, time, dt, [tcp], cb=log_error)
    time = loop_until_stop(ci, model, time, dt)

    robot.sense()
    tcp_pose = robot.model().getPose(tcp_name)
    goal_pos.append(tcp_pose.translation)
    goal_pos_des.append(tcp_tgt.translation)

    i += 1

print('saving logs')
from scipy.io import savemat


from datetime import datetime
now = datetime.now() 
date_time = now.strftime("%Y_%m_%d__%H_%M_%S")
savemat('/tmp/pick_and_place_log_' + date_time + '.mat', 
    {
        'goal_pos_des': goal_pos_des,
        'goal_pos': goal_pos,
        'ee_pos': ee_pos,
        'ee_pos_des': ee_pos_des,
        'ee_rot': ee_rot,
        'ee_rot_err': ee_rot_err,
    })


