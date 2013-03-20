#!/usr/bin/env python

import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
from rave_experimental.msg import *

class Broadcaster():
	""" Simple testing program that allows joint values to be entered in and sent to OpenRAVE using ROS """
	
	def __init__(self):
		self.joint_control_pub = rospy.Publisher('rave_joint_control_experimental', JointUpdate)
		self.joint_request_pub = rospy.Publisher('rave_joint_request_experimental', JointRequest)
		rospy.Subscriber("rave_joint_state_experimental", JointState, self.joint_request_callback)
		rospy.init_node('manual_control')
	
	def send_update(self, joint0, joint1, joint2, joint3, joint4, join5, joint6, claw_close):
		""" Publishes an update for the arm """
		self.joint_control_pub.publish(JointUpdate(joint0, joint1, joint2, joint3, joint4, joint5, joint6, claw_close))
	
	def request_joints(self):
		""" Make a request for the current joint values of the arm """
		self.joint_request_pub.publish(JointRequest())
	
	def joint_request_callback(self, msg):
		""" Displays the joint state as reported by OpenRAVE """
		print "\n%f, %f, %f, %f, %f, %f, %f, %f\n" % (msg.joint0, msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6, msg.claw_close)
		    
if __name__ == '__main__':
    try:
        broadcaster = Broadcaster()
        finished = False
        while not finished:
        	joint0 = float(raw_input("Joint 0:"))
        	joint1 = float(raw_input("Joint 1:"))
        	joint2 = float(raw_input("Joint 2:"))
        	joint3 = float(raw_input("Joint 3:"))
        	joint4 = float(raw_input("Joint 4:"))
        	joint5 = float(raw_input("Joint 5:"))
        	joint6 = float(raw_input("Joint 6:"))
        	claw_close = float(raw_input("Claw close (1/0):"))
        	
        	broadcaster.send_update(joint0, joint1, joint2, joint3, joint4, joint5, joint6, claw_close)
        	print "Send update to arm..."
        	
        	finished = raw_input("Finished? (y/n)") == "y"
        	
        	broadcaster.request_joints()
        	time.sleep(1)
        	
    except rospy.ROSInterruptException:
    	print "Failed to start ROS broadcasting"
