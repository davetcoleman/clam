#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_hardware_interface')

import rospy
from dynamixel_hardware_interface.srv import LoadController
from dynamixel_hardware_interface.srv import UnloadController
from dynamixel_hardware_interface.srv import RestartController


if __name__ == '__main__':
    try:
        usage_msg = 'Usage: %prog [options] CONTROLLER_NAMES'
        desc_msg = 'Allows to start, stop or restart joint controllers'
        epi_msg = 'Example: %s --manager=dxl_manager --port=dxl_port pan_controller tilt_controller' % sys.argv[0]
        
        parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
        parser.add_option('-m', '--manager', metavar='MANAGER',
                          help='specified serial port is managed by MANAGER')
        parser.add_option('-p', '--port', metavar='PORT',
                          help='motors of specified controllers are connected to PORT')
        parser.add_option('-c', '--command', metavar='COMMAND', default='start', choices=('start','stop','restart'),
                          help='command to perform on specified controllers: start, stop or restart [default: %default]')
                          
        (options, args) = parser.parse_args(rospy.myargv()[1:])
        
        if len(args) < 1:
            parser.error('specify at least one controller name')
            
        manager_namespace = options.manager
        port_namespace = options.port
        command = options.command
        joint_controllers = args
        
        start_service_name = '%s/load_controller' % manager_namespace
        stop_service_name = '%s/unload_controller' % manager_namespace
        restart_service_name = '%s/reload_controller' % manager_namespace
        
        rospy.init_node('controller_spawner', anonymous=True)
        
        node_name = rospy.get_name()[1:]
        parent_namespace = rospy.get_namespace()
        
        rospy.loginfo('%s: waiting for controller_manager %s to startup in %s namespace...' % (node_name, manager_namespace, parent_namespace))
        rospy.wait_for_service(start_service_name)
        rospy.wait_for_service(stop_service_name)
        rospy.wait_for_service(restart_service_name)
        
        load_controller = rospy.ServiceProxy(start_service_name, LoadController)
        unload_controller = rospy.ServiceProxy(stop_service_name, UnloadController)
        reload_controller = rospy.ServiceProxy(restart_service_name, RestartController)
        
        rospy.loginfo('%s: all services are up, %sing %d controllers...' % (node_name, command.lower(), len(joint_controllers)))
        
        for controller_name in joint_controllers:
            try:
                if command.lower() == 'start': response = load_controller(controller_name, port_namespace)
                elif command.lower() == 'stop': response = unload_controller(controller_name)
                elif command.lower() == 'restart': response = reload_controller(controller_name)
                else: rospy.logerr("%s: invalid command '%s'" % (node_name, command)); parser.print_help(); continue
                if response: rospy.loginfo("%s: command '%s %s' completed successufully" % (node_name, command.lower(), controller_name));
            except rospy.ServiceException, e:
                rospy.logerr('%s: %s' % (node_name, e))
                    
    except rospy.ROSInterruptException: pass
