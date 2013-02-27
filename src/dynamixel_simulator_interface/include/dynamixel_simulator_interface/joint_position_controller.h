/*
  Copyright (c) 2011, Antons Rebguns <email>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of the <organization> nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef JOINT_POSITION_CONTROLLER_H
#define JOINT_POSITION_CONTROLLER_H

#include <map>
#include <string>

//#include <dynamixel_simulator_interface/dynamixel_io.h>
#include <dynamixel_simulator_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/MotorStateList.h>
#include <dynamixel_hardware_interface/SetVelocity.h>
#include <dynamixel_hardware_interface/TorqueEnable.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

namespace controller
{

class JointPositionController : public SingleJointController
{
public:
  bool initialize(std::string name,
                  std::string port_namespace,
                  dynamixel_simulator_interface::DynamixelIO* dxl_io);

  bool processTorqueEnable(dynamixel_hardware_interface::TorqueEnable::Request& req,
                           dynamixel_hardware_interface::TorqueEnable::Request& res);

  std::vector<std::vector<int> > getRawMotorCommands(double position, double velocity);

  void processMotorStates(); //const dynamixel_hardware_interface::MotorStateListConstPtr& msg);
  void processCommand(const std_msgs::Float64ConstPtr& msg);

  bool setVelocity(double velocity);
  bool processSetVelocity(dynamixel_hardware_interface::SetVelocity::Request& req,
                          dynamixel_hardware_interface::SetVelocity::Request& res);

private:
  uint16_t posRad2Enc(double pos_rad);
  uint16_t velRad2Enc(double vel_rad);
};

}

#endif  // JOINT_POSITION_CONTROLLER_H
