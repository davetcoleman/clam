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

/* Author: Dave Coleman */

/*
  Allows you to perform various actions to the clam arm: (see clam_controller::ClamArmAction::command msg)
  - Reset the arm to its default/home pose
  - Open and close the end effector
  - Close end effector with feedback
  - Shutdown: go to sleep position
*/


// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Clam
#include <clam_msgs/ClamArmAction.h>

// Dynamixel
#include <dynamixel_hardware_interface/SetVelocity.h> // For changing servo velocities using service call
#include <dynamixel_hardware_interface/TorqueEnable.h> // For changing servo velocities using service call
#include <dynamixel_hardware_interface/JointState.h> // For knowing the state of the end effector

// Messages
#include <control_msgs/FollowJointTrajectoryAction.h> // for sending arm to home position
#include <std_msgs/Float64.h> // For sending end effector joint position commands

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/plan_execution/plan_execution.h>
//#include <moveit/plan_execution/plan_with_sensing.h>
//#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
//#include <moveit/kinematics_planner/kinematics_planner.h>
//#include <moveit/trajectory_processing/iterative_smoother.h>

namespace clam_controller
{

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

// Constants for the end_effector
static const double END_EFFECTOR_OPEN_VALUE = -1.0;
static const double END_EFFECTOR_CLOSE_VALUE_MAX = -0.05; //-0.1;
static const double END_EFFECTOR_POSITION_TOLERANCE = 0.02;
static const double END_EFFECTOR_VELOCITY = 0.6;
static const double END_EFFECTOR_MEDIUM_VELOCITY = 0.4;
static const double END_EFFECTOR_SLOW_VELOCITY = 0.1;
static const double END_EFFECTOR_LOAD_SETPOINT = -0.35; // when less than this number, stop closing. original value: -0.3
static const std::string EE_VELOCITY_SRV_NAME = "/l_gripper_aft_controller/set_velocity";
static const std::string EE_STATE_MSG_NAME = "/l_gripper_aft_controller/state";
static const std::string EE_POSITION_MSG_NAME = "/l_gripper_aft_controller/command";

// Constants for MoveIt
static const std::string GROUP_NAME = "arm";
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_LINK = "gripper_roll_link";

class ClamArmServer
{
private:
  ros::NodeHandle nh_;

  // Internal action server stuff
  actionlib::SimpleActionServer<clam_msgs::ClamArmAction> action_server_;
  std::string action_name_;
  clam_msgs::ClamArmFeedback     feedback_;
  clam_msgs::ClamArmResult       result_;
  clam_msgs::ClamArmGoalConstPtr goal_;

  // External publishers and services
  ros::Publisher end_effector_pub_; // publish joint values to servos
  ros::ServiceClient velocity_client_; // change end_effector velocity
  ros::Subscriber end_effector_status_; // read the position of the end effector

  // MoveIt
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> movegroup_action_;

  // Action client for the joint trajectory action used to trigger the arm movement action
  TrajClient* trajectory_client_;

  dynamixel_hardware_interface::JointState ee_status_;

  // This disables the anything that uses a gripper
  static const bool use_gripper_ = true;

  // Remember MoveGroupGoals
  moveit_msgs::MoveGroupGoal send_home_goal_; // only compute this once
  moveit_msgs::MoveGroupGoal send_shutdown_goal_; // only compute this once

  // MoveIt Components
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:
  ClamArmServer(const std::string name) :
    //nh_("~"),
    action_server_(name, false),
    movegroup_action_("move_group", true),
    action_name_(name)
  {

    // -----------------------------------------------------------------------------------------------
    // Setup gripper functionality
    if( use_gripper_ )
    {
      // Create publishers for servo positions
      end_effector_pub_ = nh_.advertise< std_msgs::Float64 >(EE_POSITION_MSG_NAME, 1, true);

      // Get the position of the end effector
      ROS_INFO("[clam arm] Reading end effector position");
      end_effector_status_ = nh_.subscribe( EE_STATE_MSG_NAME, 1, &ClamArmServer::proccessEEStatus, this);

    }

    // -----------------------------------------------------------------------------------------------
    // Start up the trajectory client
    trajectory_client_ = new TrajClient("/clam_trajectory_controller/follow_joint_trajectory", true);
    while(!trajectory_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_INFO("[clam arm] Waiting for the joint_trajectory_action server");
    }

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    while(!movegroup_action_.waitForServer(ros::Duration(4.0))){ // wait for server to start
      ROS_INFO("[clam arm] Waiting for the move_group action server");
    }

    // -----------------------------------------------------------------------------------------------
    // Create the goals once, then shutdown monitors
    generateMoveItGoals();


    // Change the goal constraints on the servos to be less strict, so that the controllers don't die. this is a hack
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/elbow_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/shoulder_pan_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/gripper_roll_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45

    // -----------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&ClamArmServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&ClamArmServer::preemptCB, this));
    action_server_.start();

    ROS_INFO("[clam arm] ClamArm action server ready");
  }


  void generateMoveItGoals()
  {

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    tf_.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_));

    // ---------------------------------------------------------------------------------------------
    // Check planning scene monitor
    if (planning_scene_monitor_->getPlanningScene() )
    {
      //ROS_INFO("Planning scene configured");
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    }
    else
    {
      ROS_ERROR("[clam arm] Planning scene not configured");
    }

    // ---------------------------------------------------------------------------------------------
    // Wait for complete state to be recieved
    ros::Duration(0.25).sleep();

    std::vector<std::string> missing_joints;
    while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() )
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      ROS_INFO("[clam arm] Waiting for complete state...");

      // Show unpublished joints
      planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
      for(int i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM("[clam arm] Unpublished joints: " << missing_joints[i]);
    }

    // -----------------------------------------------------------------------------------------------
    // Create the joint_state_group needed for creating the constraint
    const planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();
    const robot_model::JointModelGroup *joint_model_group = planning_scene->getRobotModel()->getJointModelGroup(GROUP_NAME);
    robot_state::RobotState robot_state = planning_scene->getCurrentState();

    robot_state::JointStateGroup joint_state_group(&robot_state, joint_model_group);

    // -----------------------------------------------------------------------------------------------
    // Create MoveGroupGoal for going home
    //joint_state_group.setToDefaultValues();  // sets to zeros
    std::map<std::string, double> joint_state_map;
    joint_state_map["elbow_pitch_joint"] = 0.0;
    joint_state_map["elbow_roll_joint"] = 0.0;
    joint_state_map["gripper_roll_joint"] = 0.0;
    joint_state_map["shoulder_pan_joint"] = 0.0;
    joint_state_map["shoulder_pitch_joint"] = 0.0;
    joint_state_map["wrist_pitch_joint"] = 0.0; //1.6622;
    joint_state_map["wrist_roll_joint"] = 0.0;
    joint_state_group.setVariableValues(joint_state_map);

    const double TOLERANCE_BELOW = 0.01;
    const double TOLERANCE_ABOVE = 0.01;
    moveit_msgs::Constraints goal_constraints =
      kinematic_constraints::constructGoalConstraints(&joint_state_group, TOLERANCE_BELOW, TOLERANCE_ABOVE);

    send_home_goal_.request.group_name = GROUP_NAME;
    send_home_goal_.request.num_planning_attempts = 1;
    send_home_goal_.request.allowed_planning_time = 5.0; // fix for moveit updates?  ros::Duration(5.0);
    send_home_goal_.request.goal_constraints.resize(1);
    send_home_goal_.request.goal_constraints[0] = goal_constraints;

    // -----------------------------------------------------------------------------------------------
    // Create MoveGroupGoal for shutting down

    //    joint_state_map["elbow_pitch_joint"] = -0.23521038747579834;
    joint_state_map["elbow_pitch_joint"] = -0.28521038747579834;
    joint_state_map["elbow_roll_joint"] = 0.06135923151542565;
    joint_state_map["gripper_roll_joint"] = -0.0051132692929521375;
    joint_state_map["shoulder_pan_joint"] = 0.0409061543436171;
    joint_state_map["shoulder_pitch_joint"] = 1.8261673124389464;
    joint_state_map["wrist_pitch_joint"] = -0.15339807878856412;
    joint_state_map["wrist_roll_joint"] = -0.0609061543436171;
    //    joint_state_map["wrist_roll_joint"] = -0.0409061543436171;
    joint_state_group.setVariableValues(joint_state_map);

    goal_constraints =
      kinematic_constraints::constructGoalConstraints(&joint_state_group, TOLERANCE_BELOW, TOLERANCE_ABOVE);

    send_shutdown_goal_.request.group_name = GROUP_NAME;
    send_shutdown_goal_.request.num_planning_attempts = 1;
    send_shutdown_goal_.request.allowed_planning_time = 5.0; // fix for moveit update? ros::Duration(5.0);
    send_shutdown_goal_.request.goal_constraints.resize(1);
    send_shutdown_goal_.request.goal_constraints[0] = goal_constraints;

    // -----------------------------------------------------------------------------------------------
    // Shutdown monitors
    planning_scene_monitor_->stopStateMonitor();
    planning_scene_monitor_.reset();
    tf_.reset();
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    goal_ = action_server_.acceptNewGoal();

    switch( goal_->command )
    {
    case clam_msgs::ClamArmGoal::RESET:
      ROS_INFO("[clam arm] Received reset arm goal");
      if( sendHome() )
      {
        ROS_INFO("[clam arm] Succeeded in sending arm home");
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
      else
      {
        ROS_INFO("[clam arm] Failed to send arm home");
        result_.success = false;
        action_server_.setSucceeded(result_);
      }
      break;
    case clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN:
      ROS_INFO("[clam arm] Received open end effector goal");
      if( use_gripper_ )
      {
        if( openEndEffector() )
        {
          result_.success = true;
          action_server_.setSucceeded(result_);
        }
        else
        {
          result_.success = false;
          action_server_.setSucceeded(result_);
        }
      }
      else
      {
        ROS_INFO("[clam arm] Skipped gripper command");
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
      break;
    case clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE:
      ROS_INFO("[clam arm] Received close end effector goal");
      if( use_gripper_ )
      {
        if( closeEndEffector() )
        {
          result_.success = true;
          action_server_.setSucceeded(result_);
        }
        else
        {
          result_.success = false;
          action_server_.setSucceeded(result_);
        }
      }
      else
      {
        ROS_INFO("[clam arm] Skipped gripper command");
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
      break;
    case clam_msgs::ClamArmGoal::END_EFFECTOR_SET:
      ROS_INFO("[clam arm] Received close end effector to setpoint goal");
      if( use_gripper_ )
      {
        setEndEffector(goal_->end_effector_setpoint);
      }
      else
      {
        ROS_INFO("[clam arm] Skipped gripper command");
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
      break;
    case clam_msgs::ClamArmGoal::SHUTDOWN:
      ROS_INFO("[clam arm] Received shutdown goal");
      if( sendShutdown() )
      {
        ROS_INFO("[clam arm] Succeeded in shutting down arm");
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
      else
      {
        ROS_INFO("[clam arm] Failed to shut down arm");
        result_.success = false;
        action_server_.setSucceeded(result_);
      }
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

  bool sendHome()
  {
    // -------------------------------------------------------------------------------------------
    // Plan
    ROS_INFO("[clam arm] Sending arm to home position");
    movegroup_action_.sendGoal(send_home_goal_);

    if(!movegroup_action_.waitForResult(ros::Duration(10.0)))
    {
      ROS_INFO_STREAM("[clam arm] Returned early?");
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[clam arm] Arm successfully went home.");
    }
    else
    {
      ROS_ERROR_STREAM("[clam arm] FAILED: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
      return false;
    }

    return true;
  }

  // Send arm to laying down position
  bool sendShutdown()
  {
    // -------------------------------------------------------------------------------------------
    // Plan
    ROS_INFO("[clam arm] Sending arm to shutdown position");
    movegroup_action_.sendGoal(send_shutdown_goal_);

    if(!movegroup_action_.waitForResult(ros::Duration(10.0)))
    {
      ROS_INFO_STREAM("[clam arm] Returned early?");
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[clam arm] Arm successfully shutdown.");
    }
    else
    {
      ROS_ERROR_STREAM("[clam arm] FAILED: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
      return false;
    }

    // -------------------------------------------------------------------------------------------
    // Turn off torque
    enableTorque("elbow_pitch_controller", false);
    enableTorque("elbow_roll_controller", false);
    enableTorque("gripper_roll_controller", false);
    enableTorque("shoulder_pan_controller", false);
    enableTorque("shoulder_pitch_controller", false);
    enableTorque("wrist_pitch_controller", false);
    enableTorque("wrist_roll_controller", false);
    ros::Duration(0.5).sleep();

    // -------------------------------------------------------------------------------------------
    // Turn torque back on
    enableTorque("elbow_pitch_controller", true);
    enableTorque("elbow_roll_controller", true);
    enableTorque("gripper_roll_controller", true);
    enableTorque("shoulder_pan_controller", true);
    enableTorque("shoulder_pitch_controller", true);
    enableTorque("wrist_pitch_controller", true);
    enableTorque("wrist_roll_controller", true);

    return true;
  }

  // Actually run the action
  bool sendHome_OnlyTrajectory()
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

    return true;
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
  bool openEndEffector()
  {
    // Error check - servos are alive and we've been recieving messages
    if( !endEffectorResponding() )
    {
      return false;
    }

    // Check if end effector is already open and arm is still
    if( ee_status_.target_position == END_EFFECTOR_OPEN_VALUE &&
        ee_status_.moving == false &&
        ee_status_.position > END_EFFECTOR_OPEN_VALUE + END_EFFECTOR_POSITION_TOLERANCE &&
        ee_status_.position < END_EFFECTOR_OPEN_VALUE - END_EFFECTOR_POSITION_TOLERANCE )
    {
      // Consider the ee to already be in the corret position
      ROS_INFO("[clam arm] End effector open: already in position");
      return true;
    }

    // Set the velocity for the end effector servo
    ROS_INFO("[clam arm] Setting end effector servo velocity");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    while(!velocity_client_.waitForExistence(ros::Duration(10.0)))
    {
      ROS_ERROR("[clam arm] Failed to set the end effector servo velocity via service call");
      return false;
    }
    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_ERROR("[clam arm] Failed to set the end effector servo velocity via service call");
      return false;
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
        return false;
      }
    }

    // It worked!
    ROS_INFO("[clam arm] Finished end effector action");
    return true;
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
      ROS_INFO("[clam arm] End effector already closed within tolerance, unable to close further");
      return true;
    }

    // Set the velocity for the end effector to a low value
    ROS_INFO("[clam arm] Setting end effector servo velocity low");
    velocity_client_ = nh_.serviceClient< dynamixel_hardware_interface::SetVelocity >(EE_VELOCITY_SRV_NAME);
    if( !velocity_client_.waitForExistence(ros::Duration(5.0)) )
    {
      ROS_ERROR("[clam arm] Timed out waiting for velocity client existance");
      return false;
    }

    dynamixel_hardware_interface::SetVelocity set_velocity_srv;
    set_velocity_srv.request.velocity = END_EFFECTOR_MEDIUM_VELOCITY;
    if( !velocity_client_.call(set_velocity_srv) )
    {
      ROS_ERROR("[clam arm] Failed to set the end effector servo velocity via service call");
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
      ROS_INFO_STREAM("[clam arm] Grasping with end effector - grasp number " << i + 1);

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
          if( joint_value.data < END_EFFECTOR_OPEN_VALUE )
            joint_value.data = END_EFFECTOR_OPEN_VALUE;

          ROS_DEBUG("[clam arm] Setting end effector setpoint to %f when it was %f", joint_value.data, ee_status_.position);
          end_effector_pub_.publish(joint_value);

          if( i == 0 ) // give lots of time to pull out the first time
          {
            ros::Duration(1.00).sleep();
            ROS_INFO_STREAM("Sleeping as we publish joint value " << joint_value.data);

            set_velocity_srv.request.velocity = END_EFFECTOR_SLOW_VELOCITY;
            if( !velocity_client_.call(set_velocity_srv) )
            {
              ROS_ERROR("[clam arm] Failed to set the end effector servo velocity via service call");
              return false;
            }
            ros::Duration(1.0).sleep();
          }
          break;
        }

        // Debug output
        ROS_DEBUG_STREAM("[clam arm] " << joint_value.data - END_EFFECTOR_POSITION_TOLERANCE << " < " <<
                         ee_status_.position << " < " << joint_value.data + END_EFFECTOR_POSITION_TOLERANCE
                         << " -- LOAD: " << ee_status_.load );

        // Wait an interval before checking again
        ros::Duration(CHECK_INTERVAL).sleep();

        // Check if timeout has occured
        timeout_sec -= CHECK_INTERVAL;
        if( timeout_sec <= 0 )
        {
          ROS_ERROR("[clam arm] Timeout: Unable to close end effector");
          return false;
        }
      }
    }

    // DONE
    ROS_INFO("[clam arm] Finished closing end effector action");
    return true;
  }

  // Update status of end effector
  void proccessEEStatus(const dynamixel_hardware_interface::JointState& msg)
  {
    ee_status_ = msg;
  }

  // Set the torque for a servo
  bool enableTorque(const std::string joint_name, bool enable)
  {
    ROS_DEBUG_STREAM("[clam arm] Setting torque for " << joint_name );

    std::string service_name = "/" + joint_name + "/torque_enable";
    ros::ServiceClient torque_client = nh_.serviceClient< dynamixel_hardware_interface::TorqueEnable >(service_name);
    while(!torque_client.waitForExistence(ros::Duration(10.0)))
    {
      ROS_ERROR_STREAM("[clam arm] Failed to find the service: " << service_name);
      return false;
    }
    dynamixel_hardware_interface::TorqueEnable set_torque_srv;
    set_torque_srv.request.torque_enable = enable;
    if( !torque_client.call(set_torque_srv) )
    {
      ROS_ERROR_STREAM("[clam arm] Failed to set the torque via service call to joint " << joint_name);
      return false;
    }

    return true;
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

