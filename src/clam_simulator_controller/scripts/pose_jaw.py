#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')

import time
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    pub = rospy.Publisher('l_gripper_aft_controller/command', Float64);
    rospy.init_node('pose_jaw', anonymous=True)

    command = -1.0

    while (True):
        print "Sending command", command
        pub.publish(command)
        if command == 0:
            command = -1.0
        else:
            command = 0
        time.sleep(4)

        
