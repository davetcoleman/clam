#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')

import time
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    pub = rospy.Publisher('gripper_finger_controller/command', Float64);
     rospy.init_node('pose_gripper_close', anonymous=True)
    time.sleep(1)
    print "Closing gripper"
    pub.publish(0.0)


        
