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


#ifndef DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H
#define DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H

#include <map>
#include <string>
#include <cmath>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/JointState.h>
#include <dynamixel_hardware_interface/MotorStateList.h>
#include <dynamixel_hardware_interface/SetVelocity.h>
#include <dynamixel_hardware_interface/TorqueEnable.h>
#include <dynamixel_hardware_interface/SetTorqueLimit.h>
#include <dynamixel_hardware_interface/SetComplianceMargin.h>
#include <dynamixel_hardware_interface/SetComplianceSlope.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

namespace controller
{

class SingleJointController
{
public:
  SingleJointController() {};

  virtual ~SingleJointController() {};

  virtual bool initialize(std::string name,
                          std::string port_namespace,
                          dynamixel_hardware_interface::DynamixelIO* dxl_io)
  {
    name_ = name;
    port_namespace_ = port_namespace;
    dxl_io_ = dxl_io;
    dead_time_ = 0;

    try
    {
      c_nh_ = ros::NodeHandle(nh_, name_);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s",
                name_.c_str(), e.what());
      return false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'",
                name_.c_str());
      return false;
    }

    if (!c_nh_.getParam("joint", joint_))
    {
      ROS_ERROR("%s: joint name is not specified", name_.c_str());
      return false;
    }

    joint_state_.name = joint_;
    XmlRpc::XmlRpcValue raw_motor_list;

    if (!c_nh_.getParam("motors", raw_motor_list))
    {
      ROS_ERROR("%s: no motors configuration present", name.c_str());
      return false;
    }

    if (raw_motor_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("%s: motors paramater is not a list", name_.c_str());
      return false;
    }

    int num_motors = raw_motor_list.size();
    motor_ids_.resize(num_motors);
    motor_data_.resize(num_motors);

    XmlRpc::XmlRpcValue available_ids;
    nh_.getParam("dynamixel/" + port_namespace_ + "/connected_ids", available_ids);

    if (available_ids.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("%s: connected_ids paramater is not an array", name_.c_str());
      return false;
    }

    for (int i = 0; i < num_motors; ++i)
    {
      XmlRpc::XmlRpcValue v = raw_motor_list[i];

      if (v.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("%s: motor configuration is not a map", name_.c_str());
        return false;
      }

      if (!v.hasMember("id"))
      {
        ROS_ERROR("%s: no motor id specified for motor %d", name_.c_str(), i);
        return false;
      }

      int motor_id = static_cast<int>(v["id"]);
      bool found_motor_id = false;

      for (int id = 0; id < available_ids.size(); ++id)
      {
        XmlRpc::XmlRpcValue val = available_ids[id];

        if (motor_id == static_cast<int>(val))
        {
          found_motor_id = true;
          break;
        }
      }

      if (!found_motor_id)
      {
        ROS_ERROR("%s: motor id %d is not connected to port %s or it is connected and not responding",
                  name_.c_str(), motor_id, port_namespace_.c_str());
        return false;
      }

      motor_ids_[i] = motor_id;
      motor_data_[i] = dxl_io_->getCachedParameters(motor_id);
      if (motor_data_[i] == NULL) { return false; }

      // first motor in the list is the master motor from which we take
      // all compliance parameters, initial, minimum and maximum positions
      if (i == 0)
      {
        if (!v.hasMember("init"))
        {
          ROS_ERROR("%s: no initial position specified for motor %d", name_.c_str(), motor_id);
          return false;
        }

        init_position_encoder_ = static_cast<int>(v["init"]);

        if (!v.hasMember("min"))
        {
          ROS_ERROR("%s: no minimum angle specified for motor %d", name_.c_str(), motor_id);
          return false;
        }

        min_angle_encoder_ = static_cast<int>(v["min"]);

        if (!v.hasMember("max"))
        {
          ROS_ERROR("%s: no maximum angle specified for motor %d", name_.c_str(), motor_id);
          return false;
        }

        max_angle_encoder_ = static_cast<int>(v["max"]);

        if (v.hasMember("compliance_margin")) { compliance_margin_ = static_cast<int>(v["compliance_margin"]); }
        else { compliance_margin_ = motor_data_[i]->cw_compliance_margin; }

        if (v.hasMember("compliance_slope")) { compliance_slope_ = static_cast<int>(v["compliance_slope"]); }
        else { compliance_slope_ = motor_data_[i]->cw_compliance_slope; }

        std::string prefix = "dynamixel/" + port_namespace_ + "/" +
          boost::lexical_cast<std::string>(motor_id) + "/";

        /***************** Joint velocity related parameters **********************/
        nh_.getParam(prefix + "max_velocity", motor_max_velocity_);
        c_nh_.param("max_velocity", max_velocity_, motor_max_velocity_);

        nh_.getParam(prefix + "radians_second_per_encoder_tick", velocity_per_encoder_tick_);
        min_velocity_ = velocity_per_encoder_tick_;

        if (max_velocity_ > motor_max_velocity_)
        {
          ROS_WARN("%s: requested maximum joint velocity exceeds motor capabilities (%f > %f)",
                   name_.c_str(), max_velocity_, motor_max_velocity_);
          max_velocity_ = motor_max_velocity_;
        }
        else if (max_velocity_ < min_velocity_)
        {
          ROS_WARN("%s: requested maximum joint velocity is too small (%f < %f)",
                   name_.c_str(), max_velocity_, min_velocity_);
          max_velocity_ = min_velocity_;
        }

        flipped_ = min_angle_encoder_ > max_angle_encoder_;

        nh_.getParam(prefix + "radians_per_encoder_tick", radians_per_encoder_tick_);
        nh_.getParam(prefix + "encoder_ticks_per_radian", encoder_ticks_per_radian_);

        if (flipped_)
        {
          min_angle_radians_ = (init_position_encoder_ - min_angle_encoder_) * radians_per_encoder_tick_;
          max_angle_radians_ = (init_position_encoder_ - max_angle_encoder_) * radians_per_encoder_tick_;
        }
        else
        {
          min_angle_radians_ = (min_angle_encoder_ - init_position_encoder_) * radians_per_encoder_tick_;
          max_angle_radians_ = (max_angle_encoder_ - init_position_encoder_) * radians_per_encoder_tick_;
        }

        nh_.getParam(prefix + "encoder_resolution", motor_model_max_encoder_);
        motor_model_max_encoder_ -= 1;
      }
      // slave motors, will mimic all master motor paramaters
      else
      {
        drive_mode_reversed_[motor_id] = false;
        if (v.hasMember("reversed")) { drive_mode_reversed_[motor_id] = static_cast<bool>(v["reversed"]); }
      }

      // set compliance margins and slopes for all motors controlling this joint to values
      // provided in the configuration or values from the master motor
      if (!dxl_io_->setComplianceMargins(motor_id, compliance_margin_, compliance_margin_))
      {
        ROS_ERROR("%s: unable to set complaince margins for motor %d", name_.c_str(), motor_id);
        return false;
      }

      if (!dxl_io_->setComplianceSlopes(motor_id, compliance_slope_, compliance_slope_))
      {
        ROS_ERROR("%s: unable to set complaince slopes for motor %d", name_.c_str(), motor_id);
        return false;
      }
    }

    return true;
  }

  const dynamixel_hardware_interface::JointState& getJointState() { return joint_state_; }
  dynamixel_hardware_interface::DynamixelIO* getPort() { return dxl_io_; }

  std::string getName() { return name_; }
  std::string getJointName() { return joint_; }
  std::string getPortNamespace() { return port_namespace_; }
  std::vector<int> getMotorIDs() { return motor_ids_; }
  double getMaxVelocity() { return max_velocity_; }

  virtual void start()
  {
    motor_states_sub_ = nh_.subscribe("motor_states/" + port_namespace_, 50, &SingleJointController::processMotorStates, this);
    joint_command_sub_ = c_nh_.subscribe("command", 50, &SingleJointController::processCommand, this);
    joint_state_pub_ = c_nh_.advertise<dynamixel_hardware_interface::JointState>("state", 50);
    joint_velocity_srv_ = c_nh_.advertiseService("set_velocity", &SingleJointController::processSetVelocity, this);
    torque_enable_srv_ = c_nh_.advertiseService("torque_enable", &SingleJointController::processTorqueEnable, this);
    reset_overload_error_srv_ = c_nh_.advertiseService("reset_overload_error", &SingleJointController::processResetOverloadError, this);
    set_torque_limit_srv_ = c_nh_.advertiseService("set_torque_limit", &SingleJointController::processSetTorqueLimit, this);
    set_compliance_margin_srv_ = c_nh_.advertiseService("set_compliance_margin", &SingleJointController::processSetComplianceMargin, this);
    set_compliance_slope_srv_ = c_nh_.advertiseService("set_compliance_slope", &SingleJointController::processSetComplianceSlope, this);
  }

  virtual void stop()
  {
    motor_states_sub_.shutdown();
    joint_command_sub_.shutdown();
    joint_state_pub_.shutdown();
    joint_velocity_srv_.shutdown();
    torque_enable_srv_.shutdown();
    reset_overload_error_srv_.shutdown();
    set_torque_limit_srv_.shutdown();
    set_compliance_margin_srv_.shutdown();
    set_compliance_slope_srv_.shutdown();
  }

  virtual bool processTorqueEnable(dynamixel_hardware_interface::TorqueEnable::Request& req,
                                   dynamixel_hardware_interface::TorqueEnable::Request& res)
  {
    return setTorqueEnable( req.torque_enable );    
  }

  bool setTorqueEnable( bool torque_enable )
  {
    std::vector<std::vector<int> > mcv;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      int motor_id = motor_ids_[i];

      std::vector<int> pair;
      pair.push_back(motor_id);
      pair.push_back(torque_enable);

      mcv.push_back(pair);
    }

    return dxl_io_->setMultiTorqueEnabled(mcv);
  }

  bool processResetOverloadError(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Request& res)
  {
    return resetOverloadError();    
  }

  bool resetOverloadError()
  {
    bool result = true;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      result &= dxl_io_->resetOverloadError(motor_ids_[i]);
    }

    return result;
  }

  bool processSetTorqueLimit(dynamixel_hardware_interface::SetTorqueLimit::Request& req,
                             dynamixel_hardware_interface::SetTorqueLimit::Request& res)
  {
    return setTorqueLimit( req.torque_limit );    
  }

  bool setTorqueLimit( double torque_limit )
  {
    if (torque_limit < 0)
    {
      ROS_WARN("%s: Torque limit is below minimum (%f < %f)", name_.c_str(), torque_limit, 0.0);
      torque_limit = 0.0;
    }
    else if (torque_limit > 1.0)
    {
      ROS_WARN("%s: Torque limit is above maximum (%f > %f)", name_.c_str(), torque_limit, 1.0);
      torque_limit = 1.0;
    }

    std::vector<std::vector<int> > mcv;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      int motor_id = motor_ids_[i];

      std::vector<int> pair;
      pair.push_back(motor_id);
      pair.push_back(torque_limit * dynamixel_hardware_interface::DXL_MAX_TORQUE_ENCODER);

      mcv.push_back(pair);
    }

    return dxl_io_->setMultiTorqueLimit(mcv);
  }


  bool processSetComplianceMargin(dynamixel_hardware_interface::SetComplianceMargin::Request& req,
                                  dynamixel_hardware_interface::SetComplianceMargin::Request& res)
  {
    return setComplianceMargin( req.margin );    
  }

  bool setComplianceMargin( int margin )
  {
    std::vector<std::vector<int> > mcv;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      int motor_id = motor_ids_[i];

      std::vector<int> pair;
      pair.push_back(motor_id);
      pair.push_back(margin);
      pair.push_back(margin);

      mcv.push_back(pair);
    }

    return dxl_io_->setMultiComplianceMargins(mcv);
  }

  bool processSetComplianceSlope(dynamixel_hardware_interface::SetComplianceSlope::Request& req,
                                 dynamixel_hardware_interface::SetComplianceSlope::Request& res)
  {
    return setComplianceSlope( req.slope );    
  }

  bool setComplianceSlope( int slope )
  {
    std::vector<std::vector<int> > mcv;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      int motor_id = motor_ids_[i];

      std::vector<int> pair;
      pair.push_back(motor_id);
      pair.push_back(slope);
      pair.push_back(slope);

      mcv.push_back(pair);
    }

    return dxl_io_->setMultiComplianceSlopes(mcv);
  }

  // Monitor state and determine if servos stop responding. if so, show error message and when they come back up re-initialize them
  void checkPowerFailure(dynamixel_hardware_interface::MotorState &state)
  {
    if( dead_time_ > TIME_DECLARE_MOTOR_DEAD && state.alive )
    {
      // assume power went down
      ROS_WARN_STREAM("Suspected " << dead_time_ << "s power loss. Reinitializing " << name_);
      setVelocity(current_velocity_);
    }

    // Remember current state for next time
    if( state.alive )
      dead_time_ = 0;
    else if (dead_time_ < INT_MAX)
      ++dead_time_;
  }

  virtual std::vector<std::vector<int> > getRawMotorCommands(double position, double velocity) = 0;

  virtual void processMotorStates(const dynamixel_hardware_interface::MotorStateListConstPtr& msg) = 0;
  virtual void processCommand(const std_msgs::Float64ConstPtr& msg) = 0;

  virtual bool setVelocity(double velocity) = 0;
  virtual bool processSetVelocity(dynamixel_hardware_interface::SetVelocity::Request& req,
                                  dynamixel_hardware_interface::SetVelocity::Request& res) = 0;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle c_nh_;

  std::string name_;
  std::string port_namespace_;
  dynamixel_hardware_interface::DynamixelIO* dxl_io_;

  std::string joint_;
  dynamixel_hardware_interface::JointState joint_state_;

  std::vector<int> motor_ids_;
  std::map<int, bool> drive_mode_reversed_;
  std::vector<const dynamixel_hardware_interface::DynamixelData*> motor_data_;

  static const double INIT_VELOCITY = 0.5;
  double max_velocity_;
  double min_velocity_;
  double current_velocity_;

  double min_angle_radians_;
  double max_angle_radians_;

  int init_position_encoder_;
  int min_angle_encoder_;
  int max_angle_encoder_;
  bool flipped_;
  int compliance_margin_;
  int compliance_slope_;

  static const int TIME_DECLARE_MOTOR_DEAD = 4;
  int dead_time_; // used to keep track of when motor goes down and we need to re-initialize

  double encoder_ticks_per_radian_;
  double radians_per_encoder_tick_;
  double velocity_per_encoder_tick_;
  double motor_max_velocity_;
  int motor_model_max_encoder_;

  ros::Subscriber motor_states_sub_;
  ros::Subscriber joint_command_sub_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer joint_velocity_srv_;
  ros::ServiceServer torque_enable_srv_;
  ros::ServiceServer reset_overload_error_srv_;
  ros::ServiceServer set_torque_limit_srv_;
  ros::ServiceServer set_compliance_margin_srv_;
  ros::ServiceServer set_compliance_slope_srv_;

  uint16_t convertToEncoder(double angle_in_radians)
  {
    double angle_in_encoder = angle_in_radians * encoder_ticks_per_radian_;
    angle_in_encoder = flipped_ ? init_position_encoder_ - angle_in_encoder : init_position_encoder_ + angle_in_encoder;
    return (int) (round(angle_in_encoder));
  }

  double convertToRadians(int angle_in_encoder)
  {
    double angle_in_radians = flipped_ ? init_position_encoder_ - angle_in_encoder : angle_in_encoder - init_position_encoder_;
    return angle_in_radians * radians_per_encoder_tick_;
  }

private:
  SingleJointController(const SingleJointController &c);
  SingleJointController& operator =(const SingleJointController &c);
};

}

#endif  // DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H





















