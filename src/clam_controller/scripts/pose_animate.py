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
               'l_gripper_aft_controller')
               
joint_commands = (-1.0, 0.5, 0.7, 1.0, 1.0, 1.0, 1.0, -0.9)
joint_commands2 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
joint_commands3 = (0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -0.9)
joint_commands4 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
joint_commands5 = (0.0, 2, 0.0, -0.3, 0.0, 0.0, 0.0, 1.0)

def main():

    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('pose_animate', anonymous=True)
    
    while(True):
        for i in range(len(pubs)):
            pubs[i].publish(joint_commands[i])
            print 'sending command'
            time.sleep(.1)

        print ' -------------------------------'
        time.sleep(2);

        for i in range(len(pubs)):
            pubs[i].publish(joint_commands2[i])
            print 'sending command'
            time.sleep(.1)

        time.sleep(2);

        for i in range(len(pubs)):
            pubs[i].publish(joint_commands3[i])
            print 'sending command'
            time.sleep(.1)

        time.sleep(2);

        for i in range(len(pubs)):
            pubs[i].publish(joint_commands4[i])
            print 'sending command'
            time.sleep(.1)

        time.sleep(2);

        for i in range(len(pubs)):
            pubs[i].publish(joint_commands4[i])
            print 'sending command'
            time.sleep(.1)

        time.sleep(2);


# Where the program starts                                                                                          
if __name__ == "__main__":
    try:
        main()
    # Now attempt to prevent the robot from falling limp                                                            
    except KeyboardInterrupt:
        print "done"
    except Exception as e:
        traceback.print_exc(e)
        print "done"
