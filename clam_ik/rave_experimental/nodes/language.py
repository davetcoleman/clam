#!/usr/bin/env python
import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
from rave_experimental.msg import *

import rave_experimental.arm.openraveLab as openraveLab

from nltk import pos_tag,word_tokenize
import nltk
import re


class Language:
	def __init__(self):
		""" Initalize openRave language test """
		# rave_experimental's listener.py node must be running to interface with sim
		self.pub_hand = rospy.Publisher('rave_enviroment_experimental', EnviromentUpdate)
		self.put_pub = rospy.Publisher('rave_put_command_experimental', PutCommand)
		self.hand_control_pub = rospy.Publisher('rave_hand_update_experimental', HandUpdate)
		self.hand_request_pub = rospy.Publisher('rave_hand_request_experimental', HandRequest)
		self.grab_request_pub = rospy.Publisher('rave_grab_command_experimental', GrabCommand)
		self.request_pub = rospy.Publisher('rave_body_status_request_experimental', BodyStatusRequest)
		rospy.Subscriber("rave_body_status_experimental", BodyStatus, self.body_request_callback)
		rospy.Subscriber("rave_grab_response_experimental", GrabResponse, self.grab_response_callback)
		rospy.Subscriber("rave_hand_position_experimental", HandState, self.hand_request_callback)
		rospy.Subscriber("rave_put_response_experimental", PutResponse, self.put_response_callback)
		rospy.init_node('language', anonymous=True)

		# add some named objects into the simulation
		self.pub_hand.publish(EnviromentUpdate("BLOCKred", "test_box.xml", -.3, .3, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKblue", "test_box.xml", .3, .3, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKgreen", "test_box.xml", 0, .3, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKyellow", "test_box.xml", -.15, .3, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKgray", "test_box.xml", -.3, 0, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKwhite", "test_box.xml", 0, -.3, 0, 0, 0, 0, False))
		self.pub_hand.publish(EnviromentUpdate("BLOCKblack", "test_box.xml", .3, 0, 0, 0, 0, 0, False))

		# Simple grammar for pointing at, grabbing and placing the simulation objects
		self.robot_grammar = nltk.parse_cfg("""
		S -> COMMAND | COMMAND COMMAND | COMMAND CONJ COMMAND | COMMAND COMMAND COMMAND
		P -> 'to' | 'at' | 'up' | 'down' | 'here' | 'on' | 'in' | 'beside' | 'next'
		SR -> P NP
		NP -> DET N | DET N SR
		CONJ -> 'and'
		COMMAND -> GRAB1 | REPLACE1 | POINT1

		GRAB0 -> 'grab' | 'take' | 'get' | 'pick' | PICKUP
		GRAB1 -> GRAB0 NP | 'pick' NP 'up' 
		PICKUP -> 'pick' 'up'

		REPLACE0 -> 'put' | 'place'
		REPLACE1 -> REPLACE0 NP SR
		
		POINT0 -> 'point' | 'look'
		POINT1 -> POINT0 SR

		NP -> DET N | DET ADJ N | DET ADJ ADJ N | N | ADJ N | ADJ ADJ N | NP SR
		DET -> 'the' | 'a' | 'this' | 'that' | 'an' | 'my'
		ADJ -> COLOR | SIZE
		COLOR -> 'black' | 'blue' | 'gray' | 'green' | 'red' | 'yellow' | 'white'
		SIZE -> 'small' | 'large'
		N ->  BLOCK | FLOOR | IT | 'test'
		IT -> 'it'
		BLOCK -> 'box' | 'cube' | 'block'
		FLOOR -> 'floor' | 'ground' | 'down'
		""")
		#Example rules:
		#COMMAND -> V | V P NP | V NP
		#COMMAND -> GRAB1 | POINT | TRAVEL | REPLACE | DROP
		#DROP -> PUT N FLOOR | 'drop' NP | 'release' NP
		#TRAVEL -> 'stop' | 'halt' | 'go' | 'come'
		
		# initialize the rest of the language model
		self.parser = nltk.ChartParser(self.robot_grammar)
		self.block_size = 0.15
		self.it = "redBLOCK"  #assume this is the first target
		self.waiting = False
		self.picked_up = False
		self.body_x = 0
		self.body_y = 0
		self.body_z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.closed = False
		self.error=0
	
	def wait(self):
		""" Used in conjunction with outbound messages to listener.py """
		self.waiting = True
		# timeout if waiting too long
		i = 500
		while self.waiting and i > 0:
			time.sleep(0.1)
			i-=1
			
	def grab_response_callback(self, info):
		""" Wait for the results of grab message """
		if info.errno != 0:
			print "error in grabbing %s. Error num: %d" % (info.object_id, info.errno)
		time.sleep(0.5)
		self.waiting = False	

	def hand_request_callback(self, msg):
		""" Displays the joint state as reported by OpenRAVE """
		print "\n%f, %f, %f, %f, %f, %f, closed:%s\n" % (msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, str(msg.closed))
		self.body_x = msg.x
		self.body_y = msg.y
		self.body_z = msg.z
		self.error=msg.errno
		# stop blocking for a result
		time.sleep(0.5)
		self.waiting = False
		
	def request_hand(self, body):
		""" Publishes an update for the arm and waits for a result """
		print
		self.hand_request_pub.publish(HandRequest())
			
	def body_request_callback(self, msg):
		""" Displays the body state as reported by OpenRAVE """
		print "Picked up:%s\nx:%f\ny:%f\nz:%f\nerrno:%f" % (msg.picked_up, msg.x, msg.y, msg.z, msg.errno)
		self.picked_up = msg.picked_up
		self.body_x = msg.x
		self.body_y = msg.y
		self.body_z = msg.z
		self.error=msg.errno
		time.sleep(0.5)
		self.waiting = False

	def request_body(self, body):
		""" Publishes an update for the arm and waits for a result """
		self.request_pub.publish(BodyStatusRequest(body))
		

	def send_update(self, x, y, z, roll, pitch, yaw, close):
		""" Publishes an update for the arm """
		self.hand_control_pub.publish(HandUpdate(x, y, z, roll, pitch, yaw, close))

	def place(self, object_id, x, y, z, roll, pitch, yaw, release):
		""" Publishes an update for the arm and waits for a result """
		self.put_pub.publish(PutCommand(object_id, x, y, z, roll, pitch, yaw, release))
		print "placing..."
		# Block until we have a result
		self.wait()
	
	def response_callback(self, msg):
		""" Displays the result reported by OpenRAVE """
		print "Response for %s = %d" % (msg.object_id, msg.errno)
		# stop blocking for a result
		time.sleep(0.5)
		self.waiting = False
		 
	def put_response_callback(self, msg):
		""" Displays the result reported by OpenRAVE """
		print "Response for %s = %d" % (msg.object_id, msg.errno)
		# stop blocking for a result
		time.sleep(0.5)
		self.waiting = False


	def grab_body(self, body):
		"""send a message to grab a named object in the sim"""
		print body
		print "\nplanning to grab the body..."
		self.request_pub.publish(BodyStatusRequest(body))
		self.wait()
		if not self.picked_up:
			print "resetting arm..."
			self.hand_control_pub.publish(HandUpdate(0.0, 0.0081, 0.713525, 0, 0, 1.570796, False))
			time.sleep(5)
			print "grabbing object..."
			self.grab_request_pub.publish(GrabCommand(body, False))
			self.wait()
		else:
			print "item is already picked up..."
		print "grabbing..."
		
		return True

	def put_body(self, body, p):
		""" send a message to place an item at a location """
		print body
		language.request_body(body)
		print "putting somewhere..."
		if self.error:
			return False
		if p in "on above":
			print "on above"
			self.place(body, 0, 0, 0.1, 1.57, -1.57, 1.57, False)
			self.place(body, 0, 0, 0.1, 1.57, -1.57, -1.57, False)
		elif p in "beside next":
			print "beside next"
			self.place(body, 0, 0.05, 0, 1.57, -1.57, 1.57, False)
			self.place(body, 0, 0.05, 0, 1.57, -1.57, -1.57, False)
		else:  #assume you want to put on the body
			print "put somewhere"
			self.place(body, 0, 0, 0.05, 1.57, -1.57, 1.57, False)
			self.place(body, 0, 0, 0.05, 1.57, -1.57, -1.57, False)
		return True

	def command_text(self):
		""" grab console input and call parsing function  """
		command=self.get_text()
		trees=self.parse_text(command)
		try:
			for tree in trees:
				if self.interpret_tree(tree):
					break
		except TypeError, e:
			"print: no compatible actions available"

	def get_text(self):
		""" grab raw console input """
		command = raw_input("Enter a command: ").lower()
		print command
		return command


	def parse_text(self, command):
		""" parse a string into possible tree structures """
		tokenized_sent = word_tokenize(command)
		print pos_tag(tokenized_sent)
		sent = command.split()
		try:
			trees = self.parser.nbest_parse(sent)
			return trees
		except ValueError, e:
			print "I'm sorry, I don't understand."
			print e


	def interpret_tree(self, tree):
		""" cause actions based on extracted tree """
		print tree
		for command in tree:
			flat_command = str(command)
			color = re.search("\(COLOR (\w+)\)", flat_command)
			color = "" if color == None else color.group(1)
			noun = re.search("\(N \((\w+)", flat_command)
			noun = "" if noun == None else noun.group(1)
			print "noun = "+noun
			print "item color = "+color
			if 'GRAB' in flat_command:
				if not self.interpret_grab(command):
					return False
			elif 'REPLACE' in flat_command:
				if not self.interpret_replace(command):
					return False
			elif 'POINT' in flat_command:
				if not self.interpret_point(command):
					return False
		return True

	def interpret_np(self, tree):
		""" extract information from noun phrase """
		spatial_relation=None
		flat_noun=str(tree)
		noun = re.search("\(N \((\w+)", flat_noun)
		noun = "" if noun == None else noun.group(1)
		print "noun: "+noun
		color = re.search("\(COLOR (\w+)\)", flat_noun)
		color = "" if color == None else color.group(1)
		print "color: "+color
		if "SR" in flat_noun:
			if len(tree)==3:
				spatial_relation=tree[2]
				print "supposedly a NP does have a DET attached - check ds4fdygdy"
			elif len(tree)==2:
				spatial_relation=tree[1]
				print "supposedly a NP does not have a DET attached - check j5k8kfi9"
				# possible bug: this triggers even if a DET is attached
		return noun, color, spatial_relation

	def interpret_sr(self, tree):
		""" Extract information from spatial relation phrase """
		flat_tree = str(tree)
		noun = re.search("\(N \((\w+)", flat_tree)
		noun = "" if noun == None else noun.group(1)
		color = re.search("\(COLOR (\w+)\)", flat_tree)
		color = "" if color == None else color.group(1)
		p = re.search("\(P (\w+)", flat_tree)
		p = "" if p == None else p.group(1)
		print "landmark = "+noun
		print "landmark color = "+color
		print "preposition = "+p
		if "SR" in str(tree[1]):
			print "This system is not built to go down spatial relation rabbit holes"
		return noun, color, p

	def interpret_grab(self, tree):
		""" interpret grab command """
		print "\n\n\nAction: GRAB"
		flat_tree=str(tree)
		if "GRAB1" in flat_tree:
			noun, color, sr = self.interpret_np(tree[0][1])
			if noun == "IT": #grab it
				if not self.grab_body(self.it):
					return False
			else:
				if not self.grab_body(noun+color):
					return False
			# system doesn't know how to deal with spatial relation sr, ignored for now
			if sr:
				print "STUB: GRAB1 doesn't know how to deal with spatial relations"
			self.it=color+noun
		return True

	def interpret_replace(self, tree):
		""" interpret replace command """
		print "\n\n\nAction: REPLACE"
		flat_tree=str(tree)
		if "REPLACE1" in flat_tree:
			noun1, color1, sr1 = self.interpret_np(tree[0][1])
			noun2, color2, p = self.interpret_sr(tree[0][2])
			if noun1 == "IT":  #put it on the blue block
				print "putting IT on object"
				if not self.grab_body(self.it):
					return False
				#put on the target
				print "placing..."
				if not self.put_body(noun2+color2, p):
					return False
				self.it=noun2+color2
			elif noun2 == "IT":  #put the red block on it
				print "putting object on IT"
				if not self.grab_body(noun1+color1):
					return False
				#put target on it
				print "placing..."
				if not self.put_body(self.it, p):
					return False
				self.it=noun1+color1
			else:
				print "putting an object with SR"
				if not self.grab_body(noun1+color1):
					return False
				#put obj1 on obj2
				print "placing..."
				if not self.put_body(noun2+color2, p):
					return False
				self.it=noun1+color1
		return True
		
	def interpret_point(self, tree):
		""" Interpret pointing command """
		print "Action: POINT"
		flat_tree=str(tree)
		if "POINT1" in flat_tree:
			noun, color, p = self.interpret_sr(tree[0][1])
			if noun == "IT": #grab it
				if not self.put_body(self.it, p):
					return False
			else:
				if not self.put_body(noun+color, p):
					return False
				self.it=color+noun
		return True

if __name__ == "__main__":
	try:
		language=Language()
		finished = False
		while not finished:
			language.command_text()
        	finished = raw_input("Finished? (y/n)") == "y"
        	time.sleep(1)
	except rospy.ROSInterruptException:
		pass

