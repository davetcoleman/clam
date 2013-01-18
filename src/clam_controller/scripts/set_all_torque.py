#!/usr/bin/env python
import roslib; roslib.load_manifest('clam_controller')
import rospy
from std_msgs.msg import Float64
from dynamixel_hardware_interface.msg import JointState
from dynamixel_hardware_interface.srv import TorqueEnable
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
        print 'Usage: set_all_torque.py <0,1>\n'
        print 'Controls if the motors are activated and have torque\n'
        print '1 is True, 0 is False\n'
        sys.exit(1)

    print 'Setting torque to '+sys.argv[1]

    ivalue  = int(sys.argv[1])

    for joint_name in joint_names:
        print 'Looking for service ', joint_name
        rospy.wait_for_service('/'+joint_name+'/torque_enable')

        try:
            torquer = rospy.ServiceProxy('/'+joint_name+'/torque_enable', TorqueEnable)
            response = torquer(ivalue)
            print 'Response from '+joint_name+':', response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

