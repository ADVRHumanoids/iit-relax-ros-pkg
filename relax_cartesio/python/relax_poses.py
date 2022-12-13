#!/usr/bin/python3

import numpy as np
import rospy
import rospkg
import scipy.io
import yaml
from os.path import join, dirname, abspath
import os
import argparse
from math import cos, sin, trunc

os.environ['XBOT_VERBOSE'] = '2'
from xbot_interface import xbot_interface as xbi
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *

parser = argparse.ArgumentParser(description='Scripted demo showcasing RELAX arm\'s ik')
parser.add_argument('--visual', '-v', required=False, action='store_true', help='Run in visual-only mode')
parser.add_argument('--rate', '-r', required=False, type=float, help='If visual mode is set, play back motion at X times normal speed')
args = parser.parse_args()

if args.rate is not None and not args.visual and args.rate > 1:
    print('--rate option requires --visual')
    exit(1)

if args.rate is None:
    args.rate = 1.0

# some utility functions
def get_ci(model, ikpb, dt):
    return pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)


def get_ikpb_basic():
    # HACK!
    ikpath = join('/home/embedded/src/xbot2_ws/ros_src/iit-relax-ros-pkg/relax_cartesio/', 'relax_arm_stack.yaml')
    with open(ikpath, 'r') as f:
        return f.read()

def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt
    model.setJointPosition(q)
    model.update()
    return q, qdot

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
rospy.init_node('relax_poses')

# get augmented kinematic model for ik
model_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'),
                        rospy.get_param('/xbotcore/robot_description_semantic'))
model = xbi.ModelInterface(model_cfg)

# get robot from standard urdf
ctrl_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'),
                        rospy.get_param('/xbotcore/robot_description_semantic'))
robot = xbi.RobotInterface(ctrl_cfg)
robot.sense()

# set control mode
robot.setControlMode(xbi.ControlMode.Position())

# set model state 
qref = robot.eigenToMap(robot.getPositionReference())
model.setJointPosition(qref)
model.update()

# visualize solution
rspub = pyci.RobotStatePublisher(model)

# get ci
dt = 0.01
ci_basic = get_ci(model, get_ikpb_basic(), dt)
ci = ci_basic

# define tasks
time = 0.0
done = False

# shared data between states
class Data:
    pass

data = Data()

# define states
class noop:
    def __call__(self):
        pass

class goto:
    def __init__(self, task, tgt, time):

        if not isinstance(task, list):
            task = [task]

        if not isinstance(tgt, list):
            tgt = [tgt]

        if not isinstance(time, list):
            time = [time]

        for taski, tgti, timei in zip(task, tgt, time):
             taski.setPoseTarget(tgti, timei)

        self.tasks = task

    def __call__(self):
        if all([t.getTaskState() == pyci.State.Online for t in self.tasks]):
            return True

class sequence:
    def __init__(self, states, next_state):
        self.states = list(states)
        self.next_state = next_state
        self.active_state = states[0]()
    def __call__(self):
        done = self.active_state()
        if done is True:
            self.states.pop(0)
            if len(self.states) == 0:
                return self.next_state()
            self.active_state = self.states[0]()
            print('done, {} remaining..'.format(len(self.states)))

class relax_demo(sequence):
    def __init__(self):

        # change ci to basic stack
        global ci
        ci = ci_basic
        ci.reset(time)
        ci.update(time, dt)

        # relevant tasks
        ee = ci.getTask('arm_control')

        # list of poses
        states = list()

        # ee start post
        ee_start = model.getPose('relax_arm1_linkEndEffector')

        def ee_off():
            ee.setActivationState(pyci.ActivationState.Disabled)
            return True

        def ee_on():
            ee.setActivationState(pyci.ActivationState.Enabled)
            return True

        T = ee_start.copy()
        T.translation[0] = 0.55
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = 0.55
        T.translation[1] = 0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = -0.55
        T.translation[1] = 0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = 0.55
        T.translation[1] = 0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = 0.55
        T.translation[1] = -0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = -0.55
        T.translation[1] = -0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))

        T = ee_start.copy()
        T.translation[0] = 0.55
        T.translation[1] = -0.35
        T.translation[2] = 0.44
        T.quaternion = [0.0, 1.0, 0.0, 0.0]

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))


        T = ee_start.copy()

        states.append(lambda T=T: goto(ee, T, 5.0))
        states.append(lambda: wait_time(1.0, lambda: True))


        sequence.__init__(self, states, lambda: wait_time(3.0, lambda: mission_complete()))


class mission_complete:
    def __init__(self):
        done = True
    def __call__(self):
        return None


class wait_tasks:
       def __init__(self, tasks, next_state):
           self.tasks = tasks
           self.next_state = next_state

       def __call__(self):
            if all([t.getTaskState() == pyci.State.Online for t in self.tasks]):
                return self.next_state()

class wait_time:
    def __init__(self, dt, next):
        self.next_state = next
        self.tend = time + dt
    def __call__(self):
        if time >= self.tend:
            return self.next_state()

class wait_converged:
    def __init__(self, next):
        self.next_state = next
    def __call__(self):
        if np.abs(model.getJointVelocity()).max() < 0.01:
            return self.next_state()

print('started looping..')
rate = rospy.Rate(1./dt * args.rate)
state = relax_demo()

while not done and not rospy.is_shutdown():

    # run state
    next_state = state()

    # run ik
    update_ik(ci, model, time, dt)
    rspub.publishTransforms('relax_poses')

    # send model state as reference
    qref = model.getJointPositionMap()
    dqref = model.eigenToMap(model.getJointVelocity())

    if not args.visual:
        robot.setPositionReference(qref)
        robot.setVelocityReference(dqref)
        robot.move()

    # switch state if required
    if next_state is not None and next_state != state:
        print('[{:.2f}] {} --> {}'.format(time, state.__class__.__name__, next_state.__class__.__name__))
        state = next_state

    # sleep
    rate.sleep()
    time += dt
