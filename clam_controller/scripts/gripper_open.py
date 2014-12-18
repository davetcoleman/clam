#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')

import time
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    pub = rospy.Publisher('gripper_finger_controller/command', Float64);
    rospy.init_node('pose_gripper_open', anonymous=True)

    time.sleep(1)

    print "Opening gripper"
    pub.publish(-1.0)


        
