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
   Desc:   Test opening and closing gripper
*/

#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <clam_msgs/ClamArmAction.h> // for controlling the gripper
#include <actionlib/client/simple_action_client.h>

#include <iostream> // For recording data
#include <fstream>

// Constants
static const std::string GROUP_NAME = "arm";
static const std::string DATA_FILE_OUTPUT = "/home/dave/ros/clam/src/clam_moveit_experimental/data/coverage_test.dat";

// Simple test program
int main(int argc, char **argv)
{
  // -----------------------------------------------------------------------------------------------
  // Initialize node
  ros::init(argc, argv, "clam_gripper_test", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO_STREAM("[gripper test] Preparing to send command to group = " << GROUP_NAME);

  // -----------------------------------------------------------------------------------------------
  // Connect to ClamArm action server
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_("clam_arm", true);
  clam_msgs::ClamArmGoal clam_arm_goal_; // sent to the clam_arm_client_server

  while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
    ROS_INFO("[gripper test] Waiting for the clam_arm action server");
  }

  // -----------------------------------------------------------------------------------------------
  // Go to home position
  ROS_INFO("[gripper test] Resetting arm to home position");
  clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
  clam_arm_client_.sendGoal(clam_arm_goal_);
  clam_arm_client_.waitForResult(ros::Duration(20.0));
  if(!clam_arm_client_.getState().isDone())
  {
     ROS_ERROR("[gripper test] Timeout: Unable to move to home position");
     return 2;
  }

  for(int i = 0; i < 20; ++i)
  {
    if( i % 2 )
    {
      // -----------------------------------------------------------------------------------------------
      // Open gripper
      ROS_INFO("[gripper test] Opening gripper");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      clam_arm_client_.waitForResult(ros::Duration(10.0)); // has a timeout
      
      // Error check
      if( !clam_arm_client_.getState().isDone() ||
          !clam_arm_client_.getResult()->success )
      {
        ROS_ERROR("[gripper test] Timeout: Unable to open end effector");
        return 2;
      }

    }
    else
    {
      // -----------------------------------------------------------------------------------------------
      // Close gripper
      ROS_INFO("[gripper test] Closing gripper");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE;
      //clam_arm_goal_.end_effector_setpoint = 0.0; // -0.1
      clam_arm_client_.sendGoal(clam_arm_goal_);
      clam_arm_client_.waitForResult(ros::Duration(10.0)); // has a timeout

      // Error check
      if( !clam_arm_client_.getState().isDone() ||
          !clam_arm_client_.getResult()->success )
      {
        ROS_ERROR("[gripper test] Timeout: Unable to close end effector");
        return 2;
      }

    }
    ROS_WARN("Sleeping");
    ros::Duration(8).sleep();
  }

  return 0;


  ROS_INFO("[gripper test] Node exiting");
  return 0;
}


