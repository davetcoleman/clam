/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CU Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \brief   Mid-level controller for end effector of ClamArm
 * \details Open, close and effort-based close
 * \author  Dave Coleman
 */


// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>

// Clam
#include <clam_msgs/ClamGripperCommandAction.h>

// Dynamixel
#include <dynamixel_hardware_interface/SetVelocity.h> // For changing servo velocities using service call
#include <dynamixel_hardware_interface/JointState.h> // For knowing the state of the end effector

// Messages
#include <std_msgs/Float64.h> // For sending end effector joint position commands

namespace clam_controller
{

// Hardware
static const double END_EFFECTOR_OPEN_VALUE_MAX = -1.0;
static const double END_EFFECTOR_CLOSE_VALUE_MAX = -0.05; //-0.1;

// Software
static const std::string EE_VELOCITY_SRV_NAME = "/l_gripper_aft_controller/set_velocity";
static const std::string EE_STATE_MSG_NAME = "/l_gripper_aft_controller/state";
static const std::string EE_POSITION_MSG_NAME = "/l_gripper_aft_controller/command";

// Perforance
static const double END_EFFECTOR_POSITION_TOLERANCE = 0.02;
static const double END_EFFECTOR_VELOCITY = 0.6;
static const double END_EFFECTOR_MEDIUM_VELOCITY = 0.4;
static const double END_EFFECTOR_SLOW_VELOCITY = 0.1;
static const double END_EFFECTOR_LOAD_SETPOINT = -0.35; // when less than this number, stop closing. original value: -0.3

class ClamGripperServer
{
private:
  ros::NodeHandle nh_;

  // Internal action server stuff
  actionlib::SimpleActionServer<clam_msgs::ClamGripperCommandAction> action_server_;
  clam_msgs::ClamGripperCommandFeedback     feedback_;
  clam_msgs::ClamGripperCommandResult       result_;
  clam_msgs::ClamGripperCommandGoalConstPtr goal_;

  // External publishers and services
  ros::Publisher end_effector_pub_; // publish joint values to servos
  ros::ServiceClient velocity_client_; // change end_effector velocity
  ros::Subscriber end_effector_status_; // read the position of the end effector

  // Tracking status of EE
  dynamixel_hardware_interface::JointState ee_status_;

public:
  ClamGripperServer(const std::string name) :
    action_server_(name, false),
    nh_("~")
  {

    // Create publishers for servo positions
    end_effector_pub_ = nh_.advertise< std_msgs::Float64 >(EE_POSITION_MSG_NAME, 1, true);

    // Get the position of the end effector
    ROS_INFO("Reading end effector position");
    end_effector_status_ = nh_.subscribe( EE_STATE_MSG_NAME, 1, &ClamGripperServer::proccessEEStatus, this);

    // -----------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&ClamGripperServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&ClamGripperServer::preemptCB, this));
    action_server_.start();

    ROS_INFO("ClamGripperCommand action server ready");
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    goal_ = action_server_.acceptNewGoal();

    if( goal_->position > 0.5 ) // Open
    {
      ROS_INFO("Received open end effector goal");

      if( openEndEffector() )
      {
        result_.reached_goal = true;
        action_server_.setSucceeded(result_);
      }
      else
      {
        result_.reached_goal = false;
        action_server_.setSucceeded(result_);
      }
    }
    else
    {
      ROS_INFO("Received close end effector goal");
      if( closeEndEffector() )
      {
        result_.reached_goal = true;
        action_server_.setSucceeded(result_);
      }
      else
      {
        result_.reached_goal = false;
        action_server_.setSucceeded(result_);
      }
    }

    /*
      case clam_msgs::ClamGripperCommandGoal::END_EFFECTOR_SET:
      ROS_INFO("Received close end effector to setpoint goal");
      setEndEffector(goal_->end_effector_setpoint);
    */
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO_STREAM("Action prempted");
    // set the action state to preempted
    action_server_.setPreempted();
  }

  //! Returns the current state of the action
  /*actionlib::SimpleClientGoalState getState()
  {
    return trajectory_client_->getState();
  }
  */

  bool endEffectorResponding()
  {
    if( ee_status_.header.stamp < ros::Time::now() - ros::Duration(1.0) )
    {
      ROS_ERROR("Unable to open end effector: servo status is expired");
      return false;
    }
    if( !ee_status_.alive )
    {
      ROS_ERROR("Unable to open end effector: servo not responding");
      return false;
    }
    return true;
  }

  // Open end effector
  bool openEndEffector()
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return false;
    }

    // Check if end effector is already open and arm is still
    if( ee_status_.target_position == END_EFFECTOR_OPEN_VALUE_MAX &&
        ee_status_.moving == false &&
        ee_status_.position > END_EFFECTOR_OPEN_VALUE_MAX + END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < END_EFFECTOR_OPEN_VALUE_MAX - END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("End effector open: already in position");
      return true;
    }

    // Set the velocity for the end effector servo
    ROS_INFO("Setting end effector servo velocity");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    while(!velocity_client_.waitForExistence(ros::Duration(10.0)))
    {
      ROS_ERROR("Failed to set the end effector servo velocity via service call");
      return false;
    }
    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_ERROR("Failed to set the end effector servo velocity via service call");
      return false;
    }

    // Publish command to servos
    std_msgs::Float64 joint_value;
    joint_value.data = END_EFFECTOR_OPEN_VALUE_MAX;
    end_effector_pub_.publish(joint_value);

    // Wait until end effector is done moving
    int timeout = 0;
    while( ee_status_.moving == true &&
           ee_status_.position > END_EFFECTOR_OPEN_VALUE_MAX + END_EFFECTOR_POSITION_TOLERANCE &&
           ee_status_.position < END_EFFECTOR_OPEN_VALUE_MAX - END_EFFECTOR_POSITION_TOLERANCE &&
           ros::ok() )
    {
      // Feedback
      feedback_.position = ee_status_.position;
      //TODO: fill in more of the feedback
      action_server_.publishFeedback(feedback_);

      // Looping
      ros::Duration(0.25).sleep();
      ++timeout;
      if( timeout > 16 )  // wait 4 seconds
      {
        ROS_ERROR("Unable to open end effector: timeout on goal position");
        return false;
      }
    }

    // It worked!
    ROS_INFO("Finished end effector action");
    return true;
  }

  // Close end effector to setpoint
  bool setEndEffector(double setpoint)
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return false;
    }

    // Check that there is a valid end effector setpoint set
    if( setpoint >= END_EFFECTOR_CLOSE_VALUE_MAX &&
        setpoint <= END_EFFECTOR_OPEN_VALUE_MAX )
    {
      ROS_ERROR_STREAM("Unable to set end effector: out of range setpoint of " <<
                       setpoint << ". Valid range is " << END_EFFECTOR_CLOSE_VALUE_MAX << " to "
                       << END_EFFECTOR_OPEN_VALUE_MAX );
      return false;
    }

    // Check if end effector is already close and arm is still
    if( ee_status_.target_position == setpoint &&
        ee_status_.moving == false &&
        ee_status_.position > setpoint + END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < setpoint - END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("End effector close: already in position");

      return true;
    }

    // Publish command to servos
    std_msgs::Float64 joint_value;
    joint_value.data = setpoint;
    end_effector_pub_.publish(joint_value);

    // Wait until end effector is done moving
    int timeout = 0;
    while( ee_status_.moving == true &&
           ee_status_.position > setpoint + END_EFFECTOR_POSITION_TOLERANCE &&
           ee_status_.position < setpoint - END_EFFECTOR_POSITION_TOLERANCE &&
           ros::ok() )
    {
      // Feedback
      feedback_.position = ee_status_.position;
      //TODO: fill in more of the feedback
      action_server_.publishFeedback(feedback_);

      ros::Duration(0.25).sleep();
      ++timeout;
      if( timeout > 16 )  // wait 4 seconds
      {
        ROS_ERROR("Unable to close end effector: timeout on goal position");

        return false;
      }
    }

    return true;
  }

  // Close end effector
  bool closeEndEffector()
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return false;
    }

    // Check if end effector is already close and arm is stil
    if( ee_status_.target_position == END_EFFECTOR_CLOSE_VALUE_MAX &&
        ee_status_.moving == false &&
        ee_status_.position > END_EFFECTOR_CLOSE_VALUE_MAX - END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < END_EFFECTOR_CLOSE_VALUE_MAX + END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the correct position
      ROS_INFO("End effector already closed within tolerance, unable to close further");
      return true;
    }

    // Set the velocity for the end effector to a low value
    ROS_INFO("Setting end effector servo velocity low");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    if( !velocity_client_.waitForExistence(ros::Duration(5.0)) )
    {
      ROS_ERROR("Timed out waiting for velocity client existance");
      return false;
    }

    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_MEDIUM_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_ERROR("Failed to set the end effector servo velocity via service call");
      return false;
    }

    std_msgs::Float64 joint_value;
    double timeout_sec = 10.0; // time before we declare an error
    const double CHECK_INTERVAL = 0.1; // how often we check the load
    const double FIRST_BACKOUT_AMOUNT = -0.25;
    const double SECOND_BACKOUT_AMOUNT = -0.0075;
    const double BACKOUT_AMOUNT[2] = {FIRST_BACKOUT_AMOUNT, SECOND_BACKOUT_AMOUNT};

    // Grasp twice - to reduce amount of slips and ensure better grip
    for(int i = 0; i < 2; ++i)
    {
      timeout_sec = 10; // reset timeout;
      ROS_INFO_STREAM("Grasping with end effector - grasp number " << i + 1);

      // Tell servos to start closing slowly to max amount
      joint_value.data = END_EFFECTOR_CLOSE_VALUE_MAX;
      end_effector_pub_.publish(joint_value);

      // Wait until end effector is done moving
      while( ee_status_.position < joint_value.data - END_EFFECTOR_POSITION_TOLERANCE ||
                                   ee_status_.position > joint_value.data + END_EFFECTOR_POSITION_TOLERANCE )
      {
        ros::spinOnce(); // Allows ros to get the latest servo message - we need the load

        // Check if load has peaked
        if( ee_status_.load < END_EFFECTOR_LOAD_SETPOINT ) // we have touched object!
        {
          // Open the gripper back up a little to reduce the amount of load.
          // the first time open it a lot to help with grasp quality
          joint_value.data = ee_status_.position + BACKOUT_AMOUNT[i];

          // Check that we haven't passed the open limit
          if( joint_value.data < END_EFFECTOR_OPEN_VALUE_MAX )
            joint_value.data = END_EFFECTOR_OPEN_VALUE_MAX;

          ROS_DEBUG("Setting end effector setpoint to %f when it was %f", joint_value.data, ee_status_.position);
          end_effector_pub_.publish(joint_value);

          if( i == 0 ) // give lots of time to pull out the first time
          {
            ros::Duration(1.00).sleep();
            ROS_INFO_STREAM("Sleeping as we publish joint value " << joint_value.data);

            set_velocity_srv.request.velocity = END_EFFECTOR_SLOW_VELOCITY;
            if( !velocity_client_.call(set_velocity_srv) )
            {
              ROS_ERROR("Failed to set the end effector servo velocity via service call");
              return false;
            }
            ros::Duration(1.0).sleep();
          }
          break;
        }

        // Debug output
        ROS_DEBUG_STREAM("" << joint_value.data - END_EFFECTOR_POSITION_TOLERANCE << " < " <<
                         ee_status_.position << " < " << joint_value.data + END_EFFECTOR_POSITION_TOLERANCE
                         << " -- LOAD: " << ee_status_.load );

        // Feedback
        feedback_.position = ee_status_.position;
        //TODO: fill in more of the feedback
        action_server_.publishFeedback(feedback_);

        // Wait an interval before checking again
        ros::Duration(CHECK_INTERVAL).sleep();

        // Check if timeout has occured
        timeout_sec -= CHECK_INTERVAL;
        if( timeout_sec <= 0 )
        {
          ROS_ERROR("Timeout: Unable to close end effector");
          return false;
        }
      }
    }

    // DONE
    ROS_INFO("Finished closing end effector action");
    return true;
  }

  // Update status of end effector
  void proccessEEStatus(const dynamixel_hardware_interface::JointState& msg)
  {
    ee_status_ = msg;
  }


};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "clam_gripper_server");

  clam_controller::ClamGripperServer server("clam_gripper_server");

  ros::spin();

  return 0;
}

