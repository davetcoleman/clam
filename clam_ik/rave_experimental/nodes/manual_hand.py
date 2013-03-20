#!/usr/bin/env python

import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
from rave_experimental.msg import *

class Broadcaster():
	""" Simple testing program that manually sends hand updates to OpenRAVE using ROS """
	
	def __init__(self):
		self.hand_control_pub = rospy.Publisher('rave_hand_update_experimental', HandUpdate)
		self.hand_request_pub = rospy.Publisher('rave_hand_request_experimental', HandRequest)
		rospy.Subscriber("rave_hand_position_experimental", HandState, self.hand_request_callback)
		rospy.init_node('manual_control')
	
	def send_update(self, x, y, z, roll, pitch, yaw, close):
		""" Publishes an update for the arm """
		self.hand_control_pub.publish(HandUpdate(x, y, z, roll, pitch, yaw, close))
	
	def request_hand(self):
		""" Make a request for the current joint values of the arm """
		self.hand_request_pub.publish(HandRequest())
	
	def hand_request_callback(self, msg):
		""" Displays the joint state as reported by OpenRAVE """
		print "\n%f, %f, %f, %f, %f, %f, closed:%s\n" % (msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, str(msg.closed))
		    
if __name__ == '__main__':
    try:
        broadcaster = Broadcaster()
        finished = False
        while not finished:
        	broadcaster.request_hand()
        	time.sleep(1)
        	
        	x = float(raw_input("x:"))
        	y = float(raw_input("y:"))
        	z = float(raw_input("z:"))
        	roll = float(raw_input("roll:"))
        	pitch = float(raw_input("pitch:"))
        	yaw = float(raw_input("yaw:"))
        	close = raw_input("Close? (y/n)") == "y"
        	
        	broadcaster.send_update(x, y, z, roll, pitch, yaw, close)
        	print "Send update to arm..."
        	
        	close = raw_input("Finished? (y/n)") == "y"
        	
    except rospy.ROSInterruptException:
    	print "Failed to start ROS broadcasting"
