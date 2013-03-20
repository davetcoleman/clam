#!/usr/bin/env python

import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
from rave_experimental.msg import *

class Broadcaster():
	""" Simple testing program that looks up the postition of a kin body through ROS """
	
	def __init__(self):
		self.request_pub = rospy.Publisher('rave_body_status_request_experimental', BodyStatusRequest)
		rospy.Subscriber("rave_body_status_experimental", BodyStatus, self.body_request_callback)
		rospy.init_node('rave_manual_body')
		self.waiting = False
	
	def request_body(self, body):
		""" Publishes an update for the arm and waits for a result """
		self.request_pub.publish(BodyStatusRequest(body))
		
		# Block until we have a result
		self.waiting = True
		while self.waiting:
			time.sleep(0.1)
	
	def body_request_callback(self, msg):
		""" Displays the body state as reported by OpenRAVE """
		print "Picked up:%s\nx:%f\ny:%f\nz:%f\nerrno:%f" % (msg.picked_up, msg.x, msg.y, msg.z, msg.errno)
		
		# stop blocking for a result
		self.waiting = False
		    
if __name__ == '__main__':
    try:
        broadcaster = Broadcaster()
        finished = False
        while not finished:
        	
        	name = raw_input("Name of object:")
        	broadcaster.request_body(name)
        	finished = raw_input("Finished? (y/n)") == "y"
        	
    except rospy.ROSInterruptException:
    	print "Failed to start ROS broadcasting"
