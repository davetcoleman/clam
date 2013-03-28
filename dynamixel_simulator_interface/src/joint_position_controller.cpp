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

#include <sstream>
#include <climits>
#include <XmlRpcValue.h>

//#include <dynamixel_simulator_interface/dynamixel_const.h>
//#include <dynamixel_simulator_interface/dynamixel_io.h>
#include <dynamixel_simulator_interface/single_joint_controller.h>
#include <dynamixel_simulator_interface/joint_position_controller.h>
#include <dynamixel_hardware_interface/MotorState.h>
#include <dynamixel_hardware_interface/SetVelocity.h>
#include <dynamixel_hardware_interface/TorqueEnable.h>
#include <dynamixel_hardware_interface/SetTorqueLimit.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

PLUGINLIB_DECLARE_CLASS(dynamixel_simulator_interface,
                        JointPositionController,
                        controller::JointPositionController,
                        controller::SingleJointController)

namespace controller
{

bool JointPositionController::initialize(std::string name,
                                         std::string port_namespace,
                                         dynamixel_simulator_interface::DynamixelIO* dxl_io)
{
  ROS_DEBUG_STREAM_NAMED(name_,"initialize(" << name << "," << port_namespace << ")");

  if (!SingleJointController::initialize(name, port_namespace, dxl_io))
  {
    return false;
  }

  /*
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
    int motor_id = motor_ids_[i];

    if (motor_data_[i]->cw_angle_limit == 0 && motor_data_[i]->ccw_angle_limit == 0)
    {
    ROS_WARN("%s: motor %d is not set to position control mode, setting motor to position control mode", name_.c_str(), motor_id);

    if (!dxl_io_->setAngleLimits(motor_id, 0, motor_model_max_encoder_))
    {
    ROS_ERROR("%s: unable to set motor %d to position control mode", name_.c_str(), motor_id);
    return false;
    }
    }

    }
  */

  return true;
}

bool JointPositionController::processTorqueEnable(dynamixel_hardware_interface::TorqueEnable::Request& req,
                                                  dynamixel_hardware_interface::TorqueEnable::Request& res)
{
  // set target position to current joint position
  // so the motor won't go crazy once torque is enabled again
  std_msgs::Float64 position;
  position.data = joint_state_.position;
  processCommand(boost::make_shared<const std_msgs::Float64>(position));

  return SingleJointController::processTorqueEnable(req, res);
}

std::vector<std::vector<int> > JointPositionController::getRawMotorCommands(double position, double velocity)
{
  uint16_t pos_enc = posRad2Enc(position);
  uint16_t vel_enc = velRad2Enc(velocity);

  std::vector<std::vector<int> > value_pairs;

  for (size_t i = 0; i < motor_ids_.size(); ++i)
  {
    int motor_id = motor_ids_[i];

    std::vector<int> pair;
    pair.push_back(motor_id);

    // master motor
    if (i == 0)
    {
      pair.push_back(pos_enc);
    }
    // slave motors calculate their positions based on
    // selected drive mode and master position
    else
    {
      if (!drive_mode_reversed_[motor_id])
      {
        pair.push_back(pos_enc);
      }
      else // reverse, e.g. smart arm double joints
      {
        pair.push_back(motor_model_max_encoder_ - pos_enc);
      }
    }

    pair.push_back(vel_enc);

    ROS_DEBUG("%s, setting position and velocity for motor %d to %d and %d", name_.c_str(), motor_id, pair[1], pair[2]);
    value_pairs.push_back(pair);
  }

  return value_pairs;
}

/*
  void JointPositionController::processMotorStates(const dynamixel_simulator_interface::MotorStateListConstPtr& msg)
  {
  dynamixel_hardware_interface::MotorState state;
  int master_id = motor_ids_[0];

  for (size_t i = 0; i < msg->motor_states.size(); ++i)
  {
  if (master_id == msg->motor_states[i].id)
  {
  state = msg->motor_states[i];
  break;
  }
  }

  if (state.id != master_id)
  {
  ROS_ERROR("%s: motor %d not found in motor states message", name_.c_str(), master_id);
  return;
  }

  joint_state_.header.stamp = ros::Time(state.timestamp);
  joint_state_.target_position = convertToRadians(state.target_position);
  joint_state_.target_velocity = ((double)state.target_velocity / dynamixel_simulator_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
  joint_state_.position = convertToRadians(state.position);
  joint_state_.velocity = ((double)state.velocity / dynamixel_simulator_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
  joint_state_.load = (double)state.load / dynamixel_simulator_interface::DXL_MAX_LOAD_ENCODER;
  joint_state_.moving = state.moving;
  joint_state_.alive = state.alive;

  joint_state_pub_.publish(joint_state_);

  checkPowerFailure(state);
  }
*/

void JointPositionController::processMotorStates()
{
  ros::Rate rate(10.0); // TODO: tune?

  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (terminate_feedback_) { break; }
    }

    const double gain = 0.2;
    double error = fabs(current_position_ - desired_position_);

    // Simulation: update current position if necessary
    if( error > 0.00001 )
    {

      if( current_position_ > desired_position_ )
      {
        //ROS_DEBUG_STREAM_NAMED(name_,"Error " << error << " Current " << current_position_ << " Desired " << desired_position_ << " Change -" << gain*error);
        current_position_ -= gain*error;
      }
      else
      {
        //ROS_DEBUG_STREAM_NAMED(name_,"Error " << error << " Current " << current_position_ << " Desired " << desired_position_ << " Change +" << gain*error);
        current_position_ += gain*error;
      }
    }
    else
    {
      current_position_ = desired_position_; // round off
    }

    joint_state_.header.stamp = ros::Time::now();
    joint_state_.target_position = desired_position_; //convertToRadians(state.target_position);
    joint_state_.target_velocity = desired_velocity_; //((double)state.target_velocity / dynamixel_simulator_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.position = current_position_;
    joint_state_.velocity = current_velocity_; //((double)state.velocity / dynamixel_simulator_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.load = 0.0; //(double)state.load / dynamixel_simulator_interface::DXL_MAX_LOAD_ENCODER;
    joint_state_.moving = ( current_velocity_ > 0 ); //state.moving;
    joint_state_.alive = true; //state.alive;

    joint_state_pub_.publish(joint_state_);

    rate.sleep();
  }
}

void JointPositionController::processCommand(const std_msgs::Float64ConstPtr& msg)
{
  ROS_DEBUG_STREAM_NAMED(name_,"processCommand(" << msg->data << ")");

  // Simulator:
  double desired_position = msg->data;
  double desired_velocity = 1.0; // TODO: this is a dummy number

  setDesiredPositionVelocity(desired_position, desired_velocity);

  /*
  uint16_t pos_enc = posRad2Enc(msg->data);  

  std::vector<std::vector<int> > mcv;

  for (size_t i = 0; i < motor_ids_.size(); ++i)
  {
    int motor_id = motor_ids_[i];

    std::vector<int> pair;
    pair.push_back(motor_id);

    // master motor
    if (i == 0)
    {
      pair.push_back(pos_enc);
    }
    // slave motors calculate their positions based on
    // selected drive mode and master position
    else
    {
      if (!drive_mode_reversed_[motor_id])
      {
        pair.push_back(pos_enc);
      }
      else // reverse, e.g. smart arm double joints
      {
        pair.push_back(motor_model_max_encoder_ - pos_enc);
      }
    }

    ROS_DEBUG("%s: setting position for motor %d to %d", name_.c_str(), motor_id, pair[1]);
    mcv.push_back(pair);
  }
  */

  //  dxl_io_->setMultiPosition(mcv);
}

bool JointPositionController::setVelocity(double velocity)
{
  uint16_t vel_enc = velRad2Enc(velocity);
  std::vector<std::vector<int> > mcv;

  for (size_t i = 0; i < motor_ids_.size(); ++i)
  {
    std::vector<int> pair;

    pair.push_back(motor_ids_[i]);
    pair.push_back(vel_enc);

    ROS_DEBUG("%s, setting velocity for motor %d to %d", name_.c_str(), motor_ids_[i], vel_enc);
    mcv.push_back(pair);
  }

  // Remember velcity in case servos get reset
  current_velocity_ = velocity;

  //return dxl_io_->setMultiVelocity(mcv);
  return true;
}

bool JointPositionController::processSetVelocity(dynamixel_hardware_interface::SetVelocity::Request& req,
                                                 dynamixel_hardware_interface::SetVelocity::Request& res)
{
  return setVelocity(req.velocity);
}

uint16_t JointPositionController::posRad2Enc(double pos_rad)
{
  if (pos_rad < min_angle_radians_) { pos_rad = min_angle_radians_; }
  if (pos_rad > max_angle_radians_) { pos_rad = max_angle_radians_; }
  return convertToEncoder(pos_rad);
}

uint16_t JointPositionController::velRad2Enc(double vel_rad)
{
  if (vel_rad < min_velocity_) { vel_rad = min_velocity_; }
  if (vel_rad > max_velocity_) { vel_rad = max_velocity_; }

  return std::max<uint16_t>(1, (uint16_t) round(vel_rad / velocity_per_encoder_tick_));
}

} //namespace
