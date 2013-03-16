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
#include <XmlRpcValue.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/joint_torque_controller.h>
#include <dynamixel_hardware_interface/MotorState.h>
#include <dynamixel_hardware_interface/SetVelocity.h>
#include <dynamixel_hardware_interface/TorqueEnable.h>
#include <dynamixel_hardware_interface/SetTorqueLimit.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(dynamixel_hardware_interface,
                        JointTorqueController,
                        controller::JointTorqueController,
                        controller::SingleJointController)

namespace controller
{

bool JointTorqueController::initialize(std::string name,
                                       std::string port_namespace,
                                       dynamixel_hardware_interface::DynamixelIO* dxl_io)
{
    if (!SingleJointController::initialize(name, port_namespace, dxl_io)) { return false; }

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        setVelocity(0.0);
        
        if (motor_data_[i]->cw_angle_limit != 0 || motor_data_[i]->ccw_angle_limit != 0)
        {
            ROS_WARN("%s: motor %d is not set to torque control mode, setting motor to torque control mode", name_.c_str(), motor_ids_[i]);
            
            if (!dxl_io_->setAngleLimits(motor_ids_[i], 0, 0))
            {
                ROS_ERROR("%s: unable to set motor to torque control mode", name_.c_str());
                return false;
            }
        }
    }
    
    return true;
}

bool JointTorqueController::processTorqueEnable(dynamixel_hardware_interface::TorqueEnable::Request& req,
                                                  dynamixel_hardware_interface::TorqueEnable::Request& res)
{
    setVelocity(0.0);
    return SingleJointController::processTorqueEnable(req, res);
}

std::vector<std::vector<int> > JointTorqueController::getRawMotorCommands(double position, double velocity)
{
    int16_t vel_enc = velRad2Enc(velocity);
    std::vector<std::vector<int> > mcv;
    
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        int motor_id = motor_ids_[i];
        
        std::vector<int> pair;
        pair.push_back(motor_id);
        
        if (i == 0)
        {
            pair.push_back(vel_enc);
        }
        else
        {
            if (!drive_mode_reversed_[motor_id])
            {
                pair.push_back(vel_enc);
            }
            else
            {
                pair.push_back(-vel_enc);
            }
        }
        
        ROS_DEBUG("%s, setting velocity for motor %d to %d", name_.c_str(), motor_id, vel_enc);
        mcv.push_back(pair);
    }

    return mcv;
}

void JointTorqueController::processMotorStates(const dynamixel_hardware_interface::MotorStateListConstPtr& msg)
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
        ROS_ERROR("%s: motor id %d not found", name_.c_str(), master_id);
        return;
    }
    
    joint_state_.header.stamp = ros::Time(state.timestamp);
    joint_state_.target_position = convertToRadians(state.target_position);
    joint_state_.target_velocity = ((double)state.target_velocity / dynamixel_hardware_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.position = convertToRadians(state.position);
    joint_state_.velocity = ((double)state.velocity / dynamixel_hardware_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.load = (double)state.load / dynamixel_hardware_interface::DXL_MAX_LOAD_ENCODER;
    joint_state_.moving = state.moving;
    joint_state_.alive = state.alive;

    joint_state_pub_.publish(joint_state_);

    checkPowerFailure(state);
}

void JointTorqueController::processCommand(const std_msgs::Float64ConstPtr& msg)
{
    setVelocity(msg->data);
}

bool JointTorqueController::setVelocity(double velocity)
{
    return dxl_io_->setMultiVelocity(getRawMotorCommands(0.0, velocity));
}

bool JointTorqueController::processSetVelocity(dynamixel_hardware_interface::SetVelocity::Request& req,
                                               dynamixel_hardware_interface::SetVelocity::Request& res)
{
    return setVelocity(req.velocity);
}

int16_t JointTorqueController::velRad2Enc(double vel_rad)
{
    if (vel_rad < -max_velocity_) { vel_rad = -max_velocity_; }
    if (vel_rad > max_velocity_) { vel_rad = max_velocity_; }
    
    return (int16_t) round(vel_rad / velocity_per_encoder_tick_);
}

}
