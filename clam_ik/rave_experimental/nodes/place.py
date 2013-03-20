#!/usr/bin/env python

import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
from rave_experimental.msg import *

class Broadcaster():
	""" Simple testing program that places the currently grabbed body at the given offset from the given body """
	
	def __init__(self):
		self.request_pub = rospy.Publisher('rave_put_command_experimental', PutCommand)
		rospy.Subscriber("rave_put_response_experimental", PutResponse, self.response_callback)
		self.waiting = False
		rospy.init_node('place_test')
	
	def place(self, object_id, x, y, z, roll, pitch, yaw, release):
		""" Publishes an update for the arm and waits for a result """
		self.request_pub.publish(PutCommand(object_id, x, y, z, roll, pitch, yaw, release))
		
		# Block until we have a result
		self.waiting = True
		while self.waiting:
			time.sleep(0.1)
	
	def response_callback(self, msg):
		""" Displays the result reported by OpenRAVE """
		print "Response for %s = %d" % (msg.object_id, msg.errno)
		
		# stop blocking for a result
		self.waiting = False
		    
if __name__ == '__main__':
    try:
        broadcaster = Broadcaster()
        finished = False
        while not finished:
        	
        	name = raw_input("Name of object:")
        	x = float(raw_input("x offset:"))
        	y = float(raw_input("y offset:"))
        	z = float(raw_input("z offset:"))
        	roll = float(raw_input("roll (1.57):"))
        	pitch = float(raw_input("pitch (1.57):"))
        	yaw = float(raw_input("yaw (1.57):"))
        	release = raw_input("Release? (y/n):") == "y"
        	broadcaster.place(name, x, y, z, roll, pitch, yaw, release)
        	finished = raw_input("Finished? (y/n):") == "y"
        	
    except rospy.ROSInterruptException:
    	print "Failed to start ROS broadcasting"
