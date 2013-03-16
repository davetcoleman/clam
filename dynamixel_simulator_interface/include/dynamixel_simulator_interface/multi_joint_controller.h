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


#ifndef DYNAMIXEL_SIMULATOR_INTERFACE_MULTI_JOINT_CONTROLLER_H
#define DYNAMIXEL_SIMULATOR_INTERFACE_MULTI_JOINT_CONTROLLER_H

#include <map>
#include <string>
#include <cmath>

//#include <dynamixel_simulator_interface/dynamixel_io.h>
#include <dynamixel_simulator_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/JointState.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace controller
{

class MultiJointController
{
public:
  MultiJointController() {};
  virtual ~MultiJointController() {};

  virtual bool initialize(std::string name, std::vector<boost::shared_ptr<controller::SingleJointController> > deps)
  {
    name_ = name;
    deps_ = deps;

    try
    {
      c_nh_ = ros::NodeHandle(nh_, name_);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name_.c_str(), e.what());
      return false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name_.c_str());
      return false;
    }

    num_joints_ = deps_.size();
    joint_names_.resize(num_joints_);

    for (size_t i = 0; i < num_joints_; ++i)
    {
      std::string joint_name = deps_[i]->getJointName();
      std::string port_namespace = deps_[i]->getPortNamespace();

      joint_names_[i] = joint_name;
      joint_to_idx_[joint_name] = i;
      joint_to_controller_[joint_name] = deps_[i];
      joint_states_[joint_name] = &deps_[i]->getJointState();

      port_to_joints_[port_namespace].push_back(joint_name);
      //port_to_io_[port_namespace] = deps_[i]->getPort();

      ROS_DEBUG("Adding joint %s to controller %s on port %s", joint_name.c_str(), name.c_str(), port_namespace.c_str());
    }

    return true;
  }

  bool setAllComplianceMarginSlope( int compliance_margin, int compliance_slope )
  {
    // Loop through every joint
    for( std::vector<std::string>::const_iterator joint_it = joint_names_.begin();
         joint_it < joint_names_.end(); ++joint_it )
    {
      joint_to_controller_[ *joint_it ]->setComplianceMargin( compliance_margin );
      joint_to_controller_[ *joint_it ]->setComplianceSlope( compliance_slope );
    }
  }

  virtual void start() = 0;
  virtual void stop() = 0;

  const std::vector<boost::shared_ptr<controller::SingleJointController> >& getDependencies() { return deps_; }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle c_nh_;

  std::string name_;
  std::vector<boost::shared_ptr<controller::SingleJointController> > deps_;

  size_t num_joints_;

  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_to_idx_;

  std::map<std::string, boost::shared_ptr<controller::SingleJointController> > joint_to_controller_;
  std::map<std::string, std::vector<std::string> > port_to_joints_;
  //  std::map<std::string, dynamixel_simulator_interface::DynamixelIO*> port_to_io_;
  std::map<std::string, const dynamixel_hardware_interface::JointState*> joint_states_;

};

}

#endif  // DYNAMIXEL_SIMULATOR_INTERFACE_MULTI_JOINT_CONTROLLER_H
