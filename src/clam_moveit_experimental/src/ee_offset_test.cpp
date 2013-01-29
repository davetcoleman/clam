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
 *   * Neither the name of the CU Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Test the offset from the gripper tip
*/

#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <clam_msgs/ClamArmAction.h> // for controlling the gripper
#include <actionlib/client/simple_action_client.h>

#include <iostream> // For recording data
#include <fstream>

// Constants
static const std::string GROUP_NAME = "arm";
static const std::string DATA_FILE_OUTPUT = "/home/dave/ros/clam/src/clam_moveit_experimental/data/ee_offset_test.dat";

// Simple test program
int main(int argc, char **argv)
{
  // -----------------------------------------------------------------------------------------------
  // Initialize node
  ros::init(argc, argv, "clam_coverage_test", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO_STREAM("[coverage test] Preparing to send command to group = " << GROUP_NAME);

  // -----------------------------------------------------------------------------------------------
  // Connect to ClamArm action server
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_("clam_arm", true);
  clam_msgs::ClamArmGoal clam_arm_goal_; // sent to the clam_arm_client_server

  while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
    ROS_INFO("[coverage test] Waiting for the clam_arm action server");
  }

  // -----------------------------------------------------------------------------------------------
  // Go to home position
  ROS_INFO("[coverage test] Resetting arm to home position");
  clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
  clam_arm_client_.sendGoal(clam_arm_goal_);
  clam_arm_client_.waitForResult(ros::Duration(20.0));
  if(!clam_arm_client_.getState().isDone())
  {
    ROS_ERROR("[gripper test] Timeout: Unable to move to home position");
    return 2;
  }

  // -----------------------------------------------------------------------------------------------
  // Close gripper
  ROS_INFO("[coverage test] Closing gripper");
  clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE;
  clam_arm_client_.sendGoal(clam_arm_goal_);
  clam_arm_client_.waitForResult(ros::Duration(10.0)); // has a timeout

  // Error check
  if( !clam_arm_client_.getState().isDone() ||
      !clam_arm_client_.getResult()->success )
  {
    ROS_ERROR("[coverage test] Timeout: Unable to close end effector");
    return 2;
  }

  // -----------------------------------------------------------------------------------------------
  // Move arm
  move_group_interface::MoveGroup group(GROUP_NAME);

  double x = 0.3;
  double y = 0.0;

  // Save results to file
  std::ofstream data_file;
  data_file.open(DATA_FILE_OUTPUT.c_str());

  group.setStartStateToCurrentState();
  //  group.setEndEffectorLink("gripper_fake_tip_link");
  //  group.setEndEffectorLink("l_gripper_aft_link");

  // -----------------------------------------------------------------------------------------------
  // Loop through z range
  for( double z = 0.3; z > 0.0; z -= 0.1 ) // valid range is 0.075 to 0.025 +- .025
  {
    // -------------------------------------------------------------------------------------------
    // Create start and goal

    //group.setStartState( start_state );
    //  group.setPositionTarget(0.22222, 0, 0.2);
    group.setPositionTarget(x, y, z);
    //group.setOrientationTarget( 0.00, 0.710502, -0.01755, 0.70346 );
    ROS_INFO_STREAM("[coverage test] Planning for x:" << x << " y:" << y << " z:" << z);
    //ROS_INFO_STREAM("End effector set to " << group.getEndEffectorLink());
    //ROS_INFO_STREAM("Joint 0 has value " << group.getCurrentJointValues()[0]);

    // -------------------------------------------------------------------------------------------
    // Plan
    move_group_interface::MoveGroup::Plan plan;

    if( group.plan(plan) )
    {
      // -----------------------------------------------------------------------------------------
      // Save to file
      data_file << x << "," << y <<  "," << z << "\n";

      // -----------------------------------------------------------------------------------------
      // Execute plan
      ROS_INFO("[coverage test] Executing...");
      group.execute(plan);

      /*
      // -----------------------------------------------------------------------------------------
      // Close Gripper
      if( gripperOpen )
      {
      ROS_INFO("[coverage test] Closing gripper");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE;
      gripperOpen = false;
      }
      else
      {
      ROS_INFO("[coverage test] Opening gripper");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
      gripperOpen = true;
      }
      clam_arm_client_.sendGoal(clam_arm_goal_);
      while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();
      */

    }
    else
    {
      ROS_WARN("[coverage test] Failed to find a plan");
    }
  }


  // -----------------------------------------------------------------------------------------------
  // Close down
  data_file.close();


  ROS_INFO("[coverage test] Node exiting");
  return 0;
}


