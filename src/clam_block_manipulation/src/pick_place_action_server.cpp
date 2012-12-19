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
#include <actionlib/client/simple_action_client.h>

#include <clam_block_manipulation/PickPlaceAction.h>
#include <clam_block_manipulation/ClamArmAction.h>

#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/PoseArray.h>

namespace clam_block_manipulation
{

class PickPlaceServer
{
private:

  ros::NodeHandle nh_;

  // MoveGroup
  //  move_group_interface::MoveGroup group_;

  // Action Servers and Clients
  actionlib::SimpleActionServer<clam_block_manipulation::PickPlaceAction> as_;
  actionlib::SimpleActionClient<clam_block_manipulation::ClamArmAction> clam_arm_client_;

  // Action messages
  clam_block_manipulation::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server
  clam_block_manipulation::PickPlaceFeedback     feedback_;
  clam_block_manipulation::PickPlaceResult       result_;
  clam_block_manipulation::PickPlaceGoalConstPtr goal_;

  // Subscriber
  ros::Subscriber pick_place_sub_;

  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;

public:
  PickPlaceServer(const std::string name) :
    nh_("~"), as_(name, false),
    //    group_("arm"),
    clam_arm_client_("clam_arm", true)
  {

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO("[pick place] Waiting for the clam_arm action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&PickPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickPlaceServer::preemptCB, this));
    as_.start();
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    ROS_INFO("[pick place] Received goal -----------------------------------------------");

    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    // Change the goal constraints on the servos to be less strict, so that the controllers don't die. this is a hack
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/elbow_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/shoulder_pan_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45

    /*
    // Check if our listener has recieved a goal from the topic yet
    if (goal_->topic.length() < 1)
    {
    ROS_INFO("[pick place] Goal has already been recieved, start moving arm");
    pickAndPlace(goal_->pickup_pose, goal_->place_pose); // yes, start moving arm
    }
    else
    {
    ROS_INFO_STREAM("[pick place] Waiting for goal to be received on topic " << goal_->topic);
    pick_place_sub_ = nh_.subscribe(goal_->topic, 1, // no, wait for topic
    &PickPlaceServer::sendGoalFromTopic, this);
    }
    */


    // Skip perception
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose end_pose;

    start_pose.position.x = 0.2;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.2;

    end_pose.position.x = 0.2;
    end_pose.position.y = 0.1;
    end_pose.position.z = 0.2;

    pickAndPlace(start_pose, end_pose);
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick place] Got goal from topic! %s", goal_->topic.c_str());
    pickAndPlace(msg->poses[0], msg->poses[1]);
    pick_place_sub_.shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO("[pick place] Preempted");
    as_.setPreempted();
  }

  bool sendGoal(const geometry_msgs::Pose& pose)
  {
    move_group_interface::MoveGroup group_("arm"); // TODO: move

    // ---------------------------------------------------------------------------------------------
    // Create start pose
    group_.setStartStateToCurrentState();

    // ---------------------------------------------------------------------------------------------
    // Create goal pose - straight up
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;
    group_.setPositionTarget(x, y, z);
    group_.setOrientationTarget( 0.00, 0.710502, -0.01755, 0.70346 );
    ROS_INFO_STREAM("[pick place] Planning for x:" << x << " y:" << y << " z:" << z);

    // -------------------------------------------------------------------------------------------
    // Plan
    ROS_INFO("HERE3");
    move_group_interface::MoveGroup::Plan plan;

    ROS_INFO("HERE");
    if( group_.plan(plan) )
    {
      ROS_INFO("YAY");
    }
    else
    {
      ROS_ERROR("[pick place] Unable to plan");
      as_.setAborted(result_);
      return false;
    }
    ROS_INFO("HERE2");

    // -----------------------------------------------------------------------------------------
    // Execute plan
    ROS_INFO("[pick place] Executing...");
    group_.execute(plan);

    return true;
  }

  // Actually run the action
  void pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    geometry_msgs::Pose desired_pose = start_pose;

    // Wait for MoveArmAction to be ready ----------------------------------------------------------
    ROS_INFO("[pick place] PickAndPlace started");

    // -----------------------------------------------------------------------------------------------
    // Go to home position
    ROS_INFO("[pick place] Resetting arm to home position");
    clam_arm_goal_.command = "RESET";
    clam_arm_client_.sendGoal(clam_arm_goal_);
    clam_arm_client_.waitForResult(ros::Duration(20.0));
    //while(!clam_arm_client_.getState().isDone() && ros::ok())
    //  ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO("[pick place] Opening gripper");
    clam_arm_goal_.command = "OPEN_GRIPPER";
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Hover over block
    ROS_INFO("[pick place] Sending arm to pre-grasp position");
    desired_pose.position.z = 0.2;
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Lower over block
    ROS_INFO("[pick place] Sending arm to grasp position");
    desired_pose.position.z = desired_pose.position.z - 0.1;
    ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO("[pick place] Closing gripper");
    clam_arm_goal_.command = "CLOSE_GRIPPER";
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Move Arm to new location
    ROS_INFO("[pick place] Sending arm to new position");
    desired_pose = end_pose;
    desired_pose.position.z = 0.2;
    ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO("[pick place] Opening gripper");
    clam_arm_goal_.command = "OPEN_GRIPPER";
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Reset
    ROS_INFO("[pick place] Going to home position");
    clam_arm_goal_.command = "RESET";
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO("[pick place] Finished ------------------------------------------------");
    ROS_INFO(" ");
    as_.setSucceeded(result_);
  }

};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_action_server");

  clam_block_manipulation::PickPlaceServer server("pick_place");
  ros::spin();

  return 0;
}

