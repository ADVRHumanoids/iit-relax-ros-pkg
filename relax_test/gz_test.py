#!/usr/bin/env python3

from re import sub
import unittest
import subprocess
from os import path, environ
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import SetBool
from xbot_msgs.srv import PluginStatus
import rospy, rosgraph
import time
import signal
import warnings
import atexit

warnings.simplefilter("ignore", ResourceWarning)

class GzTest(unittest.TestCase):

    def setUp(self) -> None:

        self.gz = None
        self.xb = None

        this_dir = path.abspath(path.dirname(__file__))
        self.config_path = path.join(this_dir, '..', 'relax_config', 'relax_basic.yaml')
        self.launch_path = path.join(this_dir, '..', 'relax_gazebo', 'launch', 'relax_world.launch')

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.ros_control = rospy.ServiceProxy('/xbotcore/ros_control/state', PluginStatus)

    def tearDown(self) -> None:
        self.get_model_state.close()

    def _wait_gz(self):
        timeout = 10
        
        try:
            self.get_model_state.wait_for_service(timeout=timeout)
            print('service active')
            time.sleep(1)
        except rospy.ROSException as e:
            print(f'timeout: {e}; exit status {self.gz.poll()}')
            return False 
            
        t0 = time.time()
        while time.time()  - t0 < timeout:
            res = self.get_model_state('relax', '')
            if res.success:
                print('got relax!')
                return True 
            else:
                print('relax not spawned, retrying..')
                time.sleep(1)

        return False

    def _ros_control(self):
        timeout = 10
        try:
            self.get_model_state.wait_for_service(timeout=timeout)
            print('ros_control available')
            time.sleep(1)
            return self.ros_control().status
        except rospy.ROSException as e:
            print(f'timeout: {e}; exit status {self.xb.poll()}')
            return '' 

    def _joint_state(self):
        from xbot_msgs.msg import JointState
        timeout = 10
        msg = rospy.wait_for_message('/xbotcore/joint_states', JointState, timeout=timeout)
        self.assertEqual(len(msg.name), 41)

    def _launch_gz(self, realsense='false', velodyne='false'):
        proc = subprocess.Popen(
            args=['roslaunch', 
                  self.launch_path, 
                  'gui:=false', f'realsense:={realsense}', f'velodyne:={velodyne}'],
            #stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        return proc

    def _launch_xbot2(self):
        proc = subprocess.Popen(
            args=['xbot2-core', '--simtime', '--config', self.config_path],
            #stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return proc

    def test_launch_gz_xbot2(self):
        
        warnings.simplefilter("ignore", ResourceWarning)
        warnings.simplefilter("ignore", DeprecationWarning)

        print('launching gz..')
        self.gz = self._launch_gz()
        print('gz running')
        
        self.assertTrue(self._wait_gz())

        print('launching xbot2')
        self.xb = self._launch_xbot2()
        print('xbot2 running')

        print('checking ros_control can be activated')
        self.assertIn(self._ros_control(), ('Running', 'Starting'))
        time.sleep(1)
        self.assertEqual(self._ros_control(), 'Running')

        print('checking joint_states can be received')
        self._joint_state()

        self.xb.send_signal(signal.SIGINT)
        self.xb.wait()
        print('xbot2 closed')

        self.gz.send_signal(signal.SIGINT)
        self.gz.wait()
        print('gz closed')




if __name__ == '__main__':
    # run test on a seperate roscore
    environ['ROS_MASTER_URI'] = 'http://localhost:11322'
    roscore = subprocess.Popen('roscore -p 11322'.split())

    while not rosgraph.is_master_online():
        print('waiting for master to come alive..')
        time.sleep(1)

    rospy.init_node('centauro_test_node')

    def kill_roscore():
        roscore.send_signal(signal.SIGINT)
        roscore.wait()

    atexit.register(kill_roscore)

    unittest.main()