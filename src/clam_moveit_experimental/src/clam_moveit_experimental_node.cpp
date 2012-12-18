/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <clam_block_manipulation/ClamArmAction.h> // for controlling the gripper
#include <actionlib/client/simple_action_client.h>
// For recording data
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clam_moveit_experimental_node", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string group_name = argc > 1 ? argv[1] : "arm";
  ROS_INFO_STREAM("Preparing to send command to group = " << group_name);


  actionlib::SimpleActionClient<clam_block_manipulation::ClamArmAction> clam_arm_action_("clam_arm", true);
  clam_block_manipulation::ClamArmGoal clam_arm_goal_; // sent to the clam_arm_action_server

  // wait for clam arm action server to come up
  while(!clam_arm_action_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("[pick and place] Waiting for the clam_arm action server");
  }

  // Go to zero
  ROS_INFO("[pick and place] Resetting");
  clam_arm_goal_.command = "RESET";
  clam_arm_action_.sendGoal(clam_arm_goal_);
  while(!clam_arm_action_.getState().isDone() && ros::ok())
  {
    //ROS_INFO("[pick and place] Waiting for gripper to open");
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

  move_group_interface::MoveGroup group(group_name);

  /*
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();

    ROS_INFO_STREAM("Current\n" << current_pose);

    std::vector<geometry_msgs::Pose> goal_pose(2);
    goal_pose[0] = current_pose.pose;
    goal_pose[1] = goal_pose[0];
    goal_pose[1].position.z -= 0.1;

    ROS_INFO_STREAM("Goal\n" << goal_pose[1]);

    group.followConstraints(goal_pose);
  */

  double z = 0.2;
  std::ofstream data_file;
  data_file.open("/home/dave/ros/clam/src/clam_moveit_experimental/data/valid_points.dat");

  // Create a robot start state of all 0 joint values
  /*kinematic_state::KinematicState start_state(group.getCurrentState());
    std::vector<double> joint_state_values( group.getCurrentJointValues().size(), 0.0 );
    start_state.setStateValues(joint_state_values);*/

  group.setStartStateToCurrentState();
  group.setEndEffectorLink("camera_calibration_link");

  for( double x = 0.1; x < 0.5; x += 0.05 )
  {
    for( double y = -0.2; y < 0.2; y += 0.05 )
    {
      //group.setStartState( start_state );

      //  group.setPositionTarget(0.22222, 0, 0.2);
      group.setPositionTarget(x, y, z);
      group.setOrientationTarget( 0.00, 0.710502, -0.01755, 0.70346 );

      ROS_INFO_STREAM("Planning for x:" << x << " y:" << y << " z:" << z);
      //ROS_INFO_STREAM("End effector set to " << group.getEndEffectorLink());
      //ROS_INFO_STREAM("Joint 0 has value " << group.getCurrentJointValues()[0]);

      move_group_interface::MoveGroup::Plan plan;
      if( group.plan(plan) )
      {
        data_file << x << "," << y <<  "," << z << "\n";

        ROS_INFO("Executing...");
        group.execute(plan);

        ROS_INFO("[pick and place] Opening gripper");
        clam_arm_goal_.command = "OPEN_GRIPPER";
        clam_arm_action_.sendGoal(clam_arm_goal_);
        while(!clam_arm_action_.getState().isDone() && ros::ok())
        {
          //ROS_INFO("[pick and place] Waiting for gripper to open");
          ros::Duration(0.1).sleep();
        }

        ROS_INFO("SLEEPING");
        sleep(1);
      }
      else
      {
        ROS_WARN("Failed to find a plan");
      }
    }
  }
  data_file.close();


  ROS_INFO("Node exiting");
  return 0;
}
