/*
 * Copyright (c) 2011, Vanadium Labs LLC
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
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <clam_block_manipulation/PickAndPlaceAction.h>

#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h> // is this necessary?
#include <clam_block_manipulation/ClamArmAction.h> // for controlling the gripper
#include <geometry_msgs/PoseArray.h>

namespace clam_block_manipulation
{

class PickAndPlaceServer
{
private:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<clam_block_manipulation::PickAndPlaceAction> as_;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> client_;
  actionlib::SimpleActionClient<ClamArmAction> clam_arm_action_;

  ClamArmGoal clam_arm_goal_; // sent to the clam_arm_action_server
  clam_block_manipulation::PickAndPlaceFeedback     feedback_;
  clam_block_manipulation::PickAndPlaceResult       result_;
  clam_block_manipulation::PickAndPlaceGoalConstPtr goal_;


  ros::Subscriber pick_and_place_sub_;

  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;

public:
  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false),
    client_("move_clam_arm", true),
    clam_arm_action_("clam_arm", true)
  {

    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    ROS_INFO("[pick and place] Received goal -----------------------------------------------");

    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    // Change the goal constraints on the servos to be less strict, so that the controllers don't die
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/elbow_pitch_joint/goal", 2); // originall it was 0.45, not sure where that is set
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/shoulder_pan_joint/goal", 2); // originall it was 0.45, not sure where that is set

    // Check if our listener has recieved a goal from the topic yet
    if (goal_->topic.length() < 1)
      pickAndPlace(goal_->pickup_pose, goal_->place_pose); // yes, start moving arm
    else
      pick_and_place_sub_ = nh_.subscribe(goal_->topic, 1, // no, wait for topic
                                          &PickAndPlaceServer::sendGoalFromTopic, this);
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick and place] Got goal from topic! %s", goal_->topic.c_str());
    pickAndPlace(msg->poses[0], msg->poses[1]);
    pick_and_place_sub_.shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO("[pick and place] Preempted");
    // set the action state to preempted
    as_.setPreempted();
  }

  // Helper function for sending goals 
  bool sendGoal(arm_navigation_msgs::MoveArmGoal &goal)
  {
    client_.sendGoal(goal);
    bool finished_within_time = client_.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      client_.cancelGoal();
      ROS_ERROR("[pick and place] Timed out achieving goal");
      as_.setAborted(result_);
      return false;
    }
    else
    {
      actionlib::SimpleClientGoalState state = client_.getState();
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("[pick and place] Goal position finished. State: %s",state.toString().c_str());
      }
      else
      {
        ROS_ERROR("[pick and place] Goal position failed. State: %s",state.toString().c_str());

        switch( int(client_.getResult()->error_code.val) )
        {
        case -31:
          ROS_ERROR("[pick and place] No IK Solution");
          break;
        case -32:
          ROS_ERROR("[pick and place] Invalid link name");
          break;
        case -33:
          ROS_ERROR("[pick and place] IK Link In Collision");
          break;
        case -34:
          ROS_ERROR("[pick and place] No FK Solution");
          break;
        case -35:
          ROS_ERROR("[pick and place] Kinematics state in collision");
          break;
        default:
          ROS_ERROR_STREAM("[pick and place] Failed w/ error code (from arm_navigation_msgs/ArmNavigationErrorCodes):" 
                           << client_.getResult()->error_code.val );
        }

        as_.setAborted(result_);
        return false;
      }
    }    
    
    // Success
    return true;
  }

  // Actually run the action
  void pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    // Wait for MoveArmAction to be ready ----------------------------------------------------------
    ROS_INFO("[pick and place] Waiting for move arm action server");
    client_.waitForServer();

    // Open gripper -------------------------------------------------------------------------------
    ROS_INFO("[pick and place] Opening gripper");
    clam_arm_goal_.command = "OPEN_GRIPPER";
    clam_arm_action_.sendGoal(clam_arm_goal_);
    while(!clam_arm_action_.getState().isDone() && ros::ok())
    {
      //ROS_INFO("[pick and place] Waiting for gripper to open");
      ros::Duration(0.1).sleep();
    }

    // Create goal ---------------------------------------------------------------------------------
    arm_navigation_msgs::MoveArmGoal goal;
    goal.motion_plan_request.group_name = "clam_arm"; // corresponds to clam_planning_description.yaml group name
    goal.motion_plan_request.num_planning_attempts = 20;
    goal.motion_plan_request.planner_id = std::string("");
    goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goal.motion_plan_request.allowed_planning_time = ros::Duration(50.0);

    // Create pose template to be resused ----------------------------------------------------------
    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.link_name = "gripper_roll_link";

    desired_pose.absolute_position_tolerance.x = 0.1; // 0.1
    desired_pose.absolute_position_tolerance.y = 0.1;
    desired_pose.absolute_position_tolerance.z = 0.1;

    desired_pose.absolute_roll_tolerance = 0.02; // 0.1
    desired_pose.absolute_pitch_tolerance = 0.02;
    desired_pose.absolute_yaw_tolerance = 0.02;


    // Create Approach------------------------------------------------------------------------------

    // arm straight up
    tf::Quaternion temp;
    temp.setRPY(0,1.57,0);
    desired_pose.pose.orientation.x = 0.00; //temp.getX();
    desired_pose.pose.orientation.y = 0.710502; //temp.getY();
    desired_pose.pose.orientation.z = -0.01755; //temp.getZ();
    desired_pose.pose.orientation.w = 0.703466; //temp.getW();

    // hover over
    // these are in m units, so 40cm = .40 m
    desired_pose.pose.position.x = 0.2222066; //start_pose.position.x;
    desired_pose.pose.position.y = -0.0088;  //start_pose.position.y;
    desired_pose.pose.position.z = 0.197156; //0.2; // z_up;

    // Send command
    ROS_INFO("[pick and place] Sending arm to pre-grasp position");
    ROS_INFO_STREAM("[pick and place] Pose: \n" << desired_pose.pose );
    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);
    if(!sendGoal(goal))
      return;

    ros::Duration(5).sleep();


    // Lower over block ----------------------------------------------------------------------------
    
    // drop down - only modify z axis
    desired_pose.absolute_position_tolerance.z = 0.01;
    desired_pose.pose.position.z = 0.17;

    // Send command
    ROS_INFO("[pick and place] Sending arm to grasp position");
    ROS_INFO_STREAM("[pick and place] Pose: \n" << desired_pose.pose );
    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);
    if(!sendGoal(goal))
      return;

    ros::Duration(5).sleep();


    // Close gripper -------------------------------------------------------------------------------

    ROS_INFO("[pick and place] Closing gripper");
    clam_arm_goal_.command = "CLOSE_GRIPPER";
    clam_arm_action_.sendGoal(clam_arm_goal_);
    while(!clam_arm_action_.getState().isDone() && ros::ok())
    {
      //      ROS_INFO("[pick and place] Waiting for gripper to close");
      ros::Duration(0.1).sleep();
    }

    ros::Duration(5).sleep();

    // Move Arm to new location --------------------------------------------------------------------

    /*
    // arm straight up
    tf::Quaternion temp2;
    temp2.setRPY(0,1.57,0);
    desired_pose.pose.orientation.x = temp2.getX();
    desired_pose.pose.orientation.y = temp2.getY();
    desired_pose.pose.orientation.z = temp2.getZ();
    desired_pose.pose.orientation.w = temp2.getW();

    ROS_INFO_STREAM("[pick and place] Pose orientation: " << desired_pose.pose.orientation.x << "\t" << desired_pose.pose.orientation.y << "\t"  << desired_pose.pose.orientation.z << "\t"  << desired_pose.pose.orientation.w );

    // hover over
    desired_pose.pose.position.x = end_pose.position.x;
    desired_pose.pose.position.y = end_pose.position.y;
    desired_pose.pose.position.z = 0.2; // z_up;

    ROS_INFO_STREAM("[pick and place] Pose position: " << desired_pose.pose.position.x << "\t" << desired_pose.pose.position.y << "\t"  << desired_pose.pose.position.z );

    // Send command
    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);
    ROS_INFO("[pick and place] Sending new location command to arm");
    if(!sendGoal(goal))
      return;
    */

    // Reset --------------------------------------------------------------------------------------
    ROS_INFO("[pick and place] Going to home position");
    clam_arm_goal_.command = "RESET";
    clam_arm_action_.sendGoal(clam_arm_goal_);
    while(!clam_arm_action_.getState().isDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Open gripper -------------------------------------------------------------------------------
    ROS_INFO("[pick and place] Opening gripper");
    clam_arm_goal_.command = "OPEN_GRIPPER";
    clam_arm_action_.sendGoal(clam_arm_goal_);
    while(!clam_arm_action_.getState().isDone() && ros::ok())
    {
      //ROS_INFO("[pick and place] Waiting for gripper to open");
      ros::Duration(0.1).sleep();
    }


    // Done ---------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO("[pick and place] Finished ------------------------------------------------");
    ROS_INFO(" ");
    as_.setSucceeded(result_);
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  clam_block_manipulation::PickAndPlaceServer server("pick_and_place");
  ros::spin();

  return 0;
}

