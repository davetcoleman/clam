#!/usr/bin/env python
import roslib; roslib.load_manifest('clam_controller')
import rospy
from std_msgs.msg import Float64
from dynamixel_hardware_interface.msg import JointState
from dynamixel_hardware_interface.srv import *
import time
import sys

joint_names = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_roll_controller',
               'elbow_pitch_controller',
               'wrist_roll_controller',
               'wrist_pitch_controller',
               'gripper_roll_controller',
               'l_gripper_aft_controller')

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print 'Usage: set_all_velocity.py <number>\n'
        sys.exit(1)

    print 'Setting velocity to '+sys.argv[1]

    ivalue  = float(sys.argv[1])
    service_name = 'set_velocity'

    for joint_name in joint_names:

        print 'Looking for service ', joint_name + '/' + service_name

        rospy.wait_for_service('/'+joint_name+'/'+service_name)

        try:
            service1 = rospy.ServiceProxy('/'+joint_name+'/'+service_name, SetVelocity)
            response = service1(ivalue)
            print '\tSuccess'
        except rospy.ServiceException, e:
            print "\tService call failed: %s"%e
            



