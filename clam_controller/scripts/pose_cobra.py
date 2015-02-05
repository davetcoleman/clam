#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')

import time
import rospy
from std_msgs.msg import Float64

joint_names = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_roll_controller',
               'elbow_pitch_controller',
               'wrist_roll_controller',
               'wrist_pitch_controller',
               'gripper_roll_controller',
               'gripper_finger_controller')
               
joint_commands = (0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1)


if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_cobra_pose', anonymous=True)
    
    for i in range(len(pubs)):
        time.sleep(.1)
        print 'Sending command'        
        pubs[i].publish(joint_commands[i])

        
