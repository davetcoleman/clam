#!/usr/bin/env python

# Copyright (c) 2010, Antons Rebguns
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.  * Redistributions
#     in binary form must reproduce the above copyright notice, this list of
#     conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution. # Neither the name of
#     the Willow Garage, Inc. nor the names of its contributors may be used to
#     endorse or promote products derived from this software without specific
#     prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Antons Rebguns
#
import roslib; roslib.load_manifest('clam_controller')
import rospy

from sensor_msgs.msg import JointState as JointState
from dynamixel_msgs.msg import JointState as DynamixelJointState

class JointStatesPublisher():
    def __init__(self):
        rospy.init_node('clam_joint_states_publisher', anonymous=True)
        
        self.controllers = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_roll_controller',
               'elbow_pitch_controller',
               'wrist_roll_controller',
               'wrist_pitch_controller',
               'gripper_roll_controller',
               'l_gripper_aft_controller')
                            
        self.joint_states = {}
                             
        for controller in self.controllers:
            joint_name = rospy.get_param(controller + '/joint')
            self.joint_states[joint_name] = DynamixelJointState(name=joint_name)
            
        # Start controller state subscribers
        [rospy.Subscriber(c + '/state', DynamixelJointState, self.controller_state_handler) for c in self.controllers]
        [rospy.wait_for_message(c + '/state', DynamixelJointState) for c in self.controllers]
        
        # Start publisher
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)
        
        publish_rate = rospy.get_param('~rate', 100)
        r = rospy.Rate(publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def controller_state_handler(self, msg):
        state = self.joint_states[msg.name]
        state.current_pos = msg.current_pos
        state.velocity = msg.velocity
        state.load = msg.load

    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointState()
        
        for joint, state in self.joint_states.items():
            msg.name.append(joint)
            msg.position.append(state.current_pos)
            msg.velocity.append(state.velocity)
            msg.effort.append(state.load)
            
        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatesPublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

