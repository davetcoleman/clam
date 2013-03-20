#!/usr/bin/env python

"""
Name: test_broadcast.py
Desc: Demonstrates the use of various features exposed through the OpenRAVE - ROS layer
Auth: Sam Pottinger
Note: Will require Pyglet to be installed

Controls:
	space: place block into OpenRAVE enviroment (must be done first)
	up: move block forward
	down: move block backward
	left: sidestep the block left
	right: sidestep the block right
	w: increase z value of block
	s: decreaes z value of bock
	q: increase yaw of block
	e: decrease yaw of block
	enter/return: attempt to grab block
	tab: print status of hand to terminal
"""

import pyglet
from pyglet.window import key

import roslib; roslib.load_manifest('rave_experimental')
import rospy
from rave_experimental.msg import *
import time

class Block(pyglet.window.Window):
	""" Simple block that this broadcaster is controlling """
	
	MOVE_STEP = 0.1
	
	def __init__(self, obj_id="test_body", xml_file="test_box.xml", x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
		self.obj_id = obj_id
		self.xml_file = xml_file
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw

class Broadcaster(pyglet.window.Window):
	""" Simple test broadcaster using keyboard controls """
	
	def __init__(self):
		pyglet.window.Window.__init__(self, width=300, height=300, caption="test broadcast")
		rospy.Subscriber("rave_hand_position_experimental", HandState, self.hand_callback)
		rospy.Subscriber("rave_grab_response_experimental", GrabResponse, self.grab_response)
		self.pub_hand = rospy.Publisher('rave_enviroment_experimental', EnviromentUpdate)
		self.pub_update = rospy.Publisher('rave_hand_request_experimental', HandRequest)
		self.pub_grab = rospy.Publisher("rave_grab_command_experimental", GrabCommand)
		self.block = None
		rospy.init_node('test_broadcaster')
		pyglet.app.run()
		
	def hand_callback(self, msg):
		""" Reports the location of the actuator as reported by OpenRAVE """
		print "\thand location:\t%f, %f, %f, %f, %f, %f, is closed:%s" % (msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, str(msg.closed))
	
	def grab_response(self, msg):
		""" Reports the results of a grab """
		print "\tgrab result:\t%d" % msg.errno
		self.waiting_for_response = False
	
	def send_update(self, grab=False):
		""" Publishes an update for the block """
		self.pub_hand.publish(EnviromentUpdate(self.block.obj_id, self.block.xml_file, self.block.x, self.block.y, self.block.z, self.block.roll, self.block.pitch, self.block.yaw, False)) # The last argument indicates if the positions should be relative to the camera position
		if grab:
			self.pub_grab.publish(GrabCommand(self.block.obj_id, True)) # the bool arg specifies if a fine guess is used (false for naive)
			self.waiting_for_response = True
		
	def request_hand(self):
		""" Request hand position """
		self.pub_update.publish(HandRequest())
	
	def make_block(self, x=0.5, y=0.5, z=0.5):
		""" Sends command to make a new sample block in the openrave sim """
		self.block = Block(x=x,y=y,z=z)
	
	def on_key_press(self, symbol, modifiers):
		""" Callback for a key press """
		
		if symbol == key.SPACE:
			self.make_block()
		elif symbol == key.UP:
			self.block.y = self.block.y + Block.MOVE_STEP
		elif symbol == key.DOWN:
			self.block.y = self.block.y - Block.MOVE_STEP
		elif symbol == key.LEFT:
			self.block.x = self.block.x - Block.MOVE_STEP
		elif symbol == key.RIGHT:
			self.block.x = self.block.x + Block.MOVE_STEP
		elif symbol == key.W:
			self.block.z = self.block.z + Block.MOVE_STEP
		elif symbol == key.S:
			self.block.z = self.block.z - Block.MOVE_STEP
		elif symbol == key.Q:
			self.block.yaw = self.block.yaw - Block.MOVE_STEP
		elif symbol == key.E:
			self.block.yaw = self.block.yaw + Block.MOVE_STEP
		elif symbol == key.RETURN:
			self.send_update(grab=True)
		elif symbol == key.TAB:
			self.request_hand()
			return # Sorry a bit of a kludge, will not send update if requesting hand
		
		self.send_update()
		    
if __name__ == '__main__':
    try:
        Broadcaster()
    except rospy.ROSInterruptException: pass
