/*
 * Copyright (c) 2012, CU Boulder
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Dave Coleman
 */

/*
  Allows you to perform various actions to the clam arm: (see clam_controller::ClamArmAction::command msg)
  - Reset the arm to its default/home pose
  - Open and close the end effector
  - TODO: close end effector with feedback
  - TODO: shutdown: go to sleep position
*/

#include <clam_controller/ClamArmAction.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> // For providing functionality
#include <actionlib/client/simple_action_client.h> // For calling the joint trajectory action
#include <dynamixel_hardware_interface/SetVelocity.h> // For changing servo velocities using service call
#include <dynamixel_hardware_interface/JointState.h> // For knowing the state of the end effector
#include <control_msgs/FollowJointTrajectoryAction.h> // for sending arm to home position
#include <std_msgs/Float64.h> // For sending end effector joint position commands

namespace clam_controller
{

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

// Constants for the end_effector
static const double END_EFFECTOR_OPEN_VALUE = -1.0;
static const double END_EFFECTOR_CLOSE_VALUE_MAX = -0.05; //-0.1;
static const double END_EFFECTOR_POSITION_TOLERANCE = 0.02;
static const double END_EFFECTOR_VELOCITY = 0.6;
static const double END_EFFECTOR_SLOW_VELOCITY = 0.1;
static const double END_EFFECTOR_LOAD_SETPOINT = -0.3;
static const std::string EE_VELOCITY_SRV_NAME = "/l_gripper_aft_controller/set_velocity";
static const std::string EE_STATE_MSG_NAME = "/l_gripper_aft_controller/state";
static const std::string EE_POSITION_MSG_NAME = "/l_gripper_aft_controller/command";

class ClamArmServer
{
private:
  ros::NodeHandle nh_;

  // Internal action server stuff
  actionlib::SimpleActionServer<clam_controller::ClamArmAction> action_server_;
  std::string action_name_;
  clam_controller::ClamArmFeedback     feedback_;
  clam_controller::ClamArmResult       result_;
  clam_controller::ClamArmGoalConstPtr goal_;

  // External publishers and services
  ros::Publisher end_effector_pub_; // publish joint values to servos
  ros::ServiceClient velocity_client_; // change end_effector velocity
  ros::Subscriber end_effector_status_; // read the position of the end effector

  // Action client for the joint trajectory action used to trigger the arm movement action
  TrajClient* trajectory_client_;

  dynamixel_hardware_interface::JointState ee_status_;

public:
  ClamArmServer(const std::string name) :
    nh_("~"),
    action_server_(name, false),
    action_name_(name)
  {

    // Create publishers for servo positions
    end_effector_pub_ = nh_.advertise< std_msgs::Float64 >(EE_POSITION_MSG_NAME, 1, true);

    // Get the position of the end effector
    ROS_INFO("[clam arm] Reading end effector position");
    end_effector_status_ = nh_.subscribe( EE_STATE_MSG_NAME, 1, &ClamArmServer::proccessEEStatus, this);

    // Start up the trajectory client
    trajectory_client_ = new TrajClient("/clam_arm_controller/follow_joint_trajectory", true);
    while(!trajectory_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("[clam arm] Waiting for the joint_trajectory_action server");
    }

    //register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&ClamArmServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&ClamArmServer::preemptCB, this));

    action_server_.start(); // This service is ready
    ROS_INFO("[clam arm] ClamArm action server ready");
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    goal_ = action_server_.acceptNewGoal();

    switch( goal_->command )
    {
    case clam_controller::ClamArmGoal::RESET:
      ROS_INFO("[clam arm] Received reset arm goal");
      resetArm();
      break;
    case clam_controller::ClamArmGoal::END_EFFECTOR_OPEN:
      ROS_INFO("[clam arm] Received open end effector goal");
      openEndEffector();
      break;
    case clam_controller::ClamArmGoal::END_EFFECTOR_CLOSE:
      ROS_INFO("[clam arm] Received close end effector goal");
      closeEndEffector();
      break;
    case clam_controller::ClamArmGoal::END_EFFECTOR_SET:
      ROS_INFO("[clam arm] Received close end effector to setpoint goal");
      setEndEffector(goal_->end_effector_setpoint);
      break;
    case clam_controller::ClamArmGoal::SHUTDOWN:
      ROS_ERROR("[clam arm] not implemented");

      break;
    default:
      ROS_ERROR_STREAM("Unknown command to clam_arm_action_server: " << goal_->command);
    }
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO("[%s] Preempted", action_name_.c_str());
    // set the action state to preempted
    action_server_.setPreempted();
  }

  // Send arm to home position
  control_msgs::FollowJointTrajectoryGoal resetTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_pitch_joint");
    goal.trajectory.joint_names.push_back("elbow_roll_joint");
    goal.trajectory.joint_names.push_back("elbow_pitch_joint");
    goal.trajectory.joint_names.push_back("wrist_roll_joint");
    goal.trajectory.joint_names.push_back("wrist_pitch_joint");
    goal.trajectory.joint_names.push_back("gripper_roll_joint");
    //    goal.trajectory.joint_names.push_back("l_gripper_aft_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);
    unsigned int num_joints = 7;

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(num_joints);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 1.5;
    goal.trajectory.points[ind].positions[6] = 0.0;
    //goal.trajectory.points[ind].positions[7] = 0.0;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(num_joints);
    for (size_t j = 0; j < num_joints; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 2.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return trajectory_client_->getState();
  }

  // Actually run the action
  void resetArm()
  {
    control_msgs::FollowJointTrajectoryGoal goal = resetTrajectory();

    ROS_INFO("[clam arm] Starting trajectory to reset arm");

    // When to start the trajectory: now
    goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);
    trajectory_client_->sendGoal(goal);

    // Wait for trajectory completion
    while(!getState().isDone() && ros::ok())
    {
      ros::Duration(0.25).sleep();
    }

    ROS_INFO("[clam arm] Finished resetting arm action");
    result_.success = true;
    action_server_.setSucceeded(result_);
  }

  bool endEffectorResponding()
  {
    if( ee_status_.header.stamp < ros::Time::now() - ros::Duration(1.0) )
    {
      ROS_ERROR("[clam arm] Unable to open end effector: servo status is expired");
      result_.success = false;
      action_server_.setSucceeded(result_);
      return false;
    }
    if( !ee_status_.alive )
    {
      ROS_ERROR("[clam arm] Unable to open end effector: servo not responding");
      result_.success = false;
      action_server_.setSucceeded(result_);
      return false;
    }
    return true;
  }

  // Open end effector
  void openEndEffector()
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return;
    }

    // Check if end effector is already open and arm is still
    if( ee_status_.target_position == END_EFFECTOR_OPEN_VALUE &&
        ee_status_.moving == false &&
        ee_status_.position > END_EFFECTOR_OPEN_VALUE + END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < END_EFFECTOR_OPEN_VALUE - END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("[clam arm] End effector open: already in position");
      result_.success = true;
      action_server_.setSucceeded(result_);
      return;
    }

    // Set the velocity for the end effector servo
    ROS_INFO("[clam arm] Setting end effector servo velocity");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    while(!velocity_client_.waitForExistence(ros::Duration(1.0)))
    {
    }
    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_WARN("[clam arm] Failed to set the end effector servo velocity via service call");
    }

    // Publish command to servos
    std_msgs::Float64 joint_value;
    joint_value.data = END_EFFECTOR_OPEN_VALUE;
    end_effector_pub_.publish(joint_value);

    // Wait until end effector is done moving
    int timeout = 0;
    while( ee_status_.moving == true &&
           ee_status_.position > END_EFFECTOR_OPEN_VALUE + END_EFFECTOR_POSITION_TOLERANCE &&
           ee_status_.position < END_EFFECTOR_OPEN_VALUE - END_EFFECTOR_POSITION_TOLERANCE &&
           ros::ok() )
    {
      ros::Duration(0.25).sleep();
      ++timeout;
      if( timeout > 16 )  // wait 4 seconds
      {
        ROS_ERROR("[clam arm] Unable to open end effector: timeout on goal position");
        result_.success = false;
        action_server_.setSucceeded(result_);
        return;
      }
    }

    // It worked!
    //    ROS_INFO("[clam arm] Finished end effector action");
    result_.success = true;
    action_server_.setSucceeded(result_);
  }

  // Close end effector to setpoint
  void setEndEffector(double setpoint)
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return;
    }

    // Check that there is a valid end effector setpoint set
    if( setpoint >= END_EFFECTOR_CLOSE_VALUE_MAX &&
        setpoint <= END_EFFECTOR_OPEN_VALUE )
    {
      ROS_ERROR_STREAM("[clam arm] Unable to set end effector: out of range setpoint of " <<
                       setpoint << ". Valid range is " << END_EFFECTOR_CLOSE_VALUE_MAX << " to "
                       << END_EFFECTOR_OPEN_VALUE );
      result_.success = false;
      action_server_.setSucceeded(result_);
      return;
    }

    // Check if end effector is already close and arm is still
    if( ee_status_.target_position == setpoint &&
        ee_status_.moving == false &&
        ee_status_.position > setpoint + END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < setpoint - END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("[clam arm] End effector close: already in position");
      result_.success = true;
      action_server_.setSucceeded(result_);
      return;
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
      ros::Duration(0.25).sleep();
      ++timeout;
      if( timeout > 16 )  // wait 4 seconds
      {
        ROS_ERROR("[clam arm] Unable to close end effector: timeout on goal position");
        result_.success = false;
        action_server_.setSucceeded(result_);
        return;
      }
    }

    // It worked!
    //    ROS_INFO("[clam arm] Finished end effector action");
    result_.success = true;
    action_server_.setSucceeded(result_);
  }

  // Close end effector
  void closeEndEffector()
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return;
    }

    // Check if end effector is already close and arm is still
    if( ee_status_.target_position == END_EFFECTOR_CLOSE_VALUE_MAX &&
        ee_status_.moving == false &&
        ee_status_.position > END_EFFECTOR_CLOSE_VALUE_MAX - END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < END_EFFECTOR_CLOSE_VALUE_MAX + END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("[clam arm] End effector already closed completely, unable to close further");
      result_.success = true;
      action_server_.setSucceeded(result_);
      return;
    }

    // Set the velocity for the end effector to a low value
    ROS_INFO("[clam arm] Setting end effector servo velocity low");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    while(!velocity_client_.waitForExistence(ros::Duration(1.0)))
    {
    }
    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_SLOW_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_WARN("[clam arm] Failed to set the end effector servo velocity via service call");
    }

    // Publish command to servos
    std_msgs::Float64 joint_value;
    joint_value.data = END_EFFECTOR_CLOSE_VALUE_MAX;
    end_effector_pub_.publish(joint_value);

    // Wait until end effector is done moving
    double timeout_sec = 10;
    double sleep_sec = 0.1;
    while( //ee_status_.moving == true &&
          ee_status_.position < joint_value.data - END_EFFECTOR_POSITION_TOLERANCE ||
          ee_status_.position > joint_value.data + END_EFFECTOR_POSITION_TOLERANCE
          //          ros::ok() )
           )
    {
      ros::spinOnce(); // Allows ros to get the latest servo message - we need the load

      // Check if load has peaked
      if( ee_status_.load < END_EFFECTOR_LOAD_SETPOINT ) // we have touched object!
      {
        joint_value.data = ee_status_.position + 0.01; // update to current position and backout a little
        ROS_INFO("Setting end effector setpoint to %f", joint_value.data);
        end_effector_pub_.publish(joint_value);
      }
      
      // Debug output
      ROS_DEBUG_STREAM(joint_value.data - END_EFFECTOR_POSITION_TOLERANCE << " < " <<
                      ee_status_.position << " < " << joint_value.data + END_EFFECTOR_POSITION_TOLERANCE 
                      << " -- LOAD: " << ee_status_.load );

      ros::Duration(sleep_sec).sleep();
      timeout_sec -= sleep_sec;
      if( timeout_sec <= 0 )
      {
        ROS_ERROR("[clam arm] Timeout: Unable to close end effector");
        result_.success = false;
        action_server_.setSucceeded(result_);
        return;
      }
    }

    // DONE
    //ROS_INFO("[clam arm] Finished end effector action --------------------------------------- \n");
    result_.success = true;
    action_server_.setSucceeded(result_);
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
  ros::init(argc, argv, "clam_arm_action_server");

  clam_controller::ClamArmServer server("clam_arm");
  ros::spin();

  return 0;
}

