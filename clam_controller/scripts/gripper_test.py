#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')
from dynamixel_hardware_interface.msg import JointState
from dynamixel_hardware_interface.srv import TorqueEnable
import time
import rospy
from std_msgs.msg import Float64

joint_names = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_roll_controller',
               'elbow_pitch_controller',
               'wrist_roll_controller',
               'wrist_pitch_controller',
               'gripper_roll_controller')

if __name__ == '__main__':

    for joint_name in joint_names:
        print 'Looking for service ', joint_name
        rospy.wait_for_service('/'+joint_name+'/torque_enable')

        try:
            torquer = rospy.ServiceProxy('/'+joint_name+'/torque_enable', TorqueEnable)
            response = torquer(0)
            print 'Response from '+joint_name+':', response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    pub = rospy.Publisher('l_gripper_aft_controller/command', Float64);
    rospy.init_node('pose_gripper_close', anonymous=True)
    time.sleep(1)
    print "Testing gripper"

    for i in range(30):
        if i % 2:
            print "Opening gripper"
            pub.publish(-1.05)
        else:
            print "Closing gripper"
            pub.publish(0.0)
        time.sleep(2)



