#!/usr/bin/env python
import roslib; roslib.load_manifest('rave_experimental')
import rospy
import time
import thread
import os
from numpy import *
from rave_experimental.msg import *
from rave_experimental.arm.errno import *

import rave_experimental.arm.openraveLab as openraveLab
#import rave_experimental.arm.visibilityplanning as visibilityplanning
from openravepy import *
from openravepy.databases import grasping, visibilitymodel
from openravepy.interfaces import BaseManipulation, TaskManipulation

SENSOR_NAME = "rgbCamera"

class State:
	""" Simple structure with x, y, z position and orientation """
	
	def __init__(self, x, y, z, roll, pitch, yaw):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw

class Listener:
	""" Simple ROS listener that will update a openrave enviroment """
	
	def __init__(self):
		
		# Initalize openrave
		self.arm = openraveLab.Arm()
		self.arm.sim()
		
		self.waiting_for_env = False
		
		# TODO: This is not extendible. Name issue needs to be resolved.
		self.sensor = self.arm.robot.GetAttachedSensors()[0]
		self.manip = [m for m in self.arm.robot.GetManipulators() if m.GetEndEffector() == self.sensor.GetAttachingLink()][0]
		
		self.Tgoals = []
		
		# Initalize visibility planning module (visibilityplanning.py)
		#self.planning = visibilityplanning.VisibilityGrasping(self.arm.env)
		
		# Initalize ROS
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("rave_enviroment_experimental", EnviromentUpdate, self.enviroment_callback)
		rospy.Subscriber("rave_hand_request_experimental", HandRequest, self.hand_request_callback)
		rospy.Subscriber("rave_joint_control_experimental", JointUpdate, self.joint_control_callback)
		rospy.Subscriber("rave_joint_request_experimental", JointRequest, self.joint_request_callback)
		rospy.Subscriber("rave_hand_update_experimental", HandUpdate, self.hand_update_callback)
		rospy.Subscriber("rave_grab_command_experimental", GrabCommand, self.grab)
		rospy.Subscriber("rave_body_status_request_experimental", BodyStatusRequest, self.body_request_callback)
		rospy.Subscriber("rave_camera_request_experimental", CameraPositionRequest, self.camera_request_callback)
		rospy.Subscriber("rave_put_command_experimental", PutCommand, self.put_command_callback)
		self.hand_pub = rospy.Publisher('rave_hand_position_experimental', HandState)
		self.joint_state_pub = rospy.Publisher('rave_joint_state_experimental', JointState)
		self.grab_response_publisher = rospy.Publisher("rave_grab_response_experimental", GrabResponse)
		self.body_status_pub = rospy.Publisher("rave_body_status_experimental", BodyStatus)
		self.camera_position_pub = rospy.Publisher("rave_camera_position_experimental", CameraPosition)
		self.put_response_pub = rospy.Publisher("rave_put_response_experimental", PutResponse)
		rospy.spin()
		
	def get_camera_position(self):
		""" Returns the camera's position / orientation as a simple stat structure """
		camera_transform = self.arm.robot.GetAttachedSensor("sensor0").GetTransform() # TODO: sensor0, really? I call myself a college CS student?
		info = openraveLab.decomposeTransform(camera_transform)
		return State(info[0], info[1], info[2], info[3], info[4], info[5])
	
	def put_command_callback(self, msg):
		""" Attempts to put whatever is currently grasped / point claw at the position of a body with a given offset """
		
		print "I am here..."
		
		# Attempt to get the body
		body = self.arm.env.GetKinBody(msg.object_id)
		if body == None:
			self.put_response_pub.publish(msg.object_id, OBJECT_NOT_LOADED)
			print "object not loaded"
			return
		
		# Offset
		body_info = openraveLab.decomposeTransform(body.GetTransform())
		x = msg.x_offset + body_info[0]
		y = msg.y_offset + body_info[1]
		z = msg.z_offset + body_info[2]
		
		# Move hand
		try:
			self.arm.moveTo(x, y, z, msg.roll, msg.pitch, msg.yaw)
			self.waitrobot()
		except:
			self.put_response_pub.publish(msg.object_id, PLANNING_FAILED)
			print "planning failed"
			return
		
		# Attempt to release
		if msg.release:
			self.arm.open()
		
		# Report back
		self.put_response_pub.publish(msg.object_id, SUCCESS)
		print "success"
	
	def camera_request_callback(self, msg):
		""" Responds to a CameraPositionRequest with a CameraPosition message """
		state = get_camera_position()
		self.camera_position_pub.publish(CameraPosition(state.x, state.y, state.z, state.roll, state.pitch, state.yaw))
	
	def body_request_callback(self, msg):
		""" Responds to a BodyStatusRequest with a BodyStatus response """
		kinbody = self.arm.env.GetKinBody(msg.object_id)
		
		if(kinbody == None):
			self.body_status_pub.publish(BodyStatus(msg.object_id, False, 0, 0, 0, OBJECT_NOT_LOADED))
		else:
			# Publish its current state
			info = openraveLab.decomposeTransform(kinbody.GetTransform())
			self.body_status_pub.publish(BodyStatus(msg.object_id, kinbody.IsAttached(self.arm.robot), info[0], info[1], info[2], SUCCESS))
	
	def joint_request_callback(self, msg):
		""" Responds to requests for the current state of the joints by publishing thier current values """
		state = self.arm.getJoints()
		self.joint_state_pub.publish(JointState(state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7]))
		
	def joint_control_callback(self, msg):
		""" Callback for the manual control of the joints of the robotic arm """
		self.arm.joints([msg.joint0, msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6, msg.claw_close, 0])
	
	def hand_request_callback(self, msg):
		""" Callback for the request topic for the location of the hand """
		arm = self.arm.env.GetKinBody("pdArm")
		transform = arm.GetJoint("JLR").GetFirstAttached().GetTransform() # Get the joint with the hand on it and then get the hand
		info = openraveLab.decomposeTransform(transform)
		self.hand_pub.publish(HandState(info[0], info[1], info[2], info[3], info[4], info[5], self.arm.fingersClosed))
	
	def hand_update_callback(self, msg):
		""" Updates the position and orientation of the hand """
		self.arm.moveTo(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)
		if msg.close:
			self.arm.close()
		else:
			self.arm.open()
	
	def enviroment_callback(self, msg):
		""" Makes an update to the openrave enviroment given a ros message """
		
		kinbody = self.arm.env.GetKinBody(msg.object_id)
		
		self.waiting_for_env = False
		
		# If there is no body by that name yet, make it
		if kinbody == None:
			kinbody = self.arm.env.ReadKinBodyXMLFile(os.path.join(os.getcwd(), "src/rave_experimental/arm/" + msg.xml_file))
			kinbody.SetName(msg.object_id)
			self.arm.env.AddKinBody(kinbody)
		
		# Offset from camera if needed
		if msg.camera_relative:
			camera_pos = self.get_camera_position()
			x = msg.x + camera_pos.x
			y = msg.y + camera_pos.y
			z = msg.z + camera_pos.z
			roll = msg.roll + camera_pos.roll
			pitch = msg.pitch + camera_pos.pitch
			yaw = msg.yaw + camera_pos.yaw
		else:
			x = msg.x
			y = msg.y
			z = msg.z
			roll = msg.roll
			pitch = msg.pitch
			yaw = msg.yaw
		
		# Update the position of the body
		kinbody.GetLinks()[0].SetTransform(openraveLab.buildTransform(x, y, z, roll, pitch, yaw))
		
		# Grab if necessary
		#if msg.grab:
			#self.planning.target = msg.object_id
			#self.planning.start()
			# Grab if necessary
			#print msg.x
			#print msg.y
			#print msg.z
			#self.arm.moveTo(msg.x, msg.y, msg.z, -1.57, 1.57, 0)
			#self.execute_visual_servoing(kinbody)
	
	def grab(self, msg):
		"""
		Response to a new GrabCommand message
		
		Response to a new GrabCommand message that will grab the given 
		object and report result through GrabResponse
		"""
		
		# Find the kinbody
		kinbody = self.arm.env.GetKinBody(msg.object_id)
		if kinbody == None:
			self.grab_response_publisher.publish(GrabResponse(msg.object_id, OBJECT_NOT_LOADED))
			return
		
		thread.start_new_thread(self.execute_visual_servoing, (kinbody, msg.object_id, msg.use_visual_feedback))
	
	def compute_visibility_model(self,target):
		SENSOR_NAME
		vmodel = visibilitymodel.VisibilityModel(robot=self.arm.robot,target=target,sensorname=self.sensor.GetName())
		if not vmodel.load():
			vmodel.autogenerate()
		return vmodel
	
	def start_visibility_trajectory(self,trajdata):
		if trajdata is not None and len(trajdata) > 0:
			#self.trajectorylog.append(trajdata)
			self.arm.b.TrajFromData(trajdata)
			self.waitrobot(self.arm.robot)
			#if self.arm.robot is not None:
			#	self.arm.robot.GetController().SetDesired(self.robotreal.GetDOFValues())
			time.sleep(0.2) # give some time for GUI to update
	
	def execute_visual_servoing(self, target, object_id, wait):
		while True:
			self.waiting_for_env = True
	
			vmodel = self.compute_visibility_model(target)
			try:
				trajdata = vmodel.visualprob.MoveToObserveTarget(sampleprob=0.001,maxiter=4000,execute=False,outputtraj=True)
			except:
				self.grab_response_publisher.publish(GrabResponse(object_id, PLANNING_FAILED))
				return
				
			self.start_visibility_trajectory(trajdata)
		
			# Update position of target
			# TODO: need to seperate out naive grasping
			if wait:
				while self.waiting_for_env:
					time.sleep(0.1)
				target = self.arm.env.GetKinBody(target.GetName())
		
			# start visual servoing step
			gmodel = grasping.GraspingModel(robot=self.arm.robot,target=target)
			if not gmodel.load():
				gmodel.autogenerate()
			taskmanip = TaskManipulation(self.arm.robot,graspername=gmodel.grasper.plannername)
			trajdata = None
			with self.arm.robot:
				validgrasps,validindices = gmodel.computeValidGrasps()
				if len(validgrasps) == 0:
					continue
				for iter in range(4):
					# set the real values for simulation
					gmodel.setPreshape(validgrasps[0])
					#trajdata = visualprob.SendCommand('VisualFeedbackGrasping target ' + self.target.GetName() + ' sensorindex 0 convexdata ' + str(self.convexdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.convexdata.flat) + ' graspsetdata ' + str(self.graspsetdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.graspsetdata[:,0:12].flat) + ' maxiter 100 visgraspthresh 0.5 gradientsamples 5 ' + cmdstr)
					try:
						trajdata = self.arm.b.MoveToHandPosition(matrices=[gmodel.getGlobalGraspTransform(g,collisionfree=True) for g in validgrasps],execute=False,outputtraj=True)
						break
					except planning_error:
						print 'trying visual feedback grasp again'
			if trajdata is None:
				continue
			self.start_visibility_trajectory(trajdata)

			try:
				final,trajdata = taskmanip.CloseFingers(execute=False)#offset=self.graspoffset*ones(len(self.manip.GetGripperIndices())),execute=False,outputtraj=True)
				self.start_visibility_trajectory(trajdata)
			except planning_error:
				self.grab_response_publisher.publish(GrabResponse(object_id, PLANNING_FAILED))
				return
			
			self.arm.robot.Grab(target)
			self.arm.fingersClosed = True
			self.grab_response_publisher.publish(GrabResponse(object_id, SUCCESS))
			
			break

				#if not success:
				#	continue

	def waitrobot(self,robot=None):
		if robot is None:
			robot = self.robotreal
		while not robot.GetController().IsDone():
			time.sleep(0.01)
			
if __name__ == '__main__':
	Listener()
