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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// ClamArm
#include <clam_msgs/ClamGripperCommandAction.h>

// Simple test program
int main(int argc, char **argv)
{
  // -----------------------------------------------------------------------------------------------
  // Initialize node
  ros::init(argc, argv, "clam_gripper_test", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO_STREAM("[gripper test] Preparing to send test the gripper");

  // -----------------------------------------------------------------------------------------------
  // Connect to Clam Gripper action server
  actionlib::SimpleActionClient<clam_msgs::ClamGripperCommandAction> clam_gripper_controller_("/clam_gripper_controller/gripper_action", true);
  clam_msgs::ClamGripperCommandGoal gripper_goal_; // sent to the clam_gripper_controller

  while(!clam_gripper_controller_.waitForServer(ros::Duration(5.0))){ // wait for server to start
    ROS_INFO("[gripper test] Waiting for the clam_gripper_controller to come up");
  }

  //for(int i = 0; i < 20; ++i)
  int i = 0;
  while(ros::ok())
  {
    if( i % 2 )
    {
      // -----------------------------------------------------------------------------------------------
      // Open gripper
      ROS_INFO("[gripper test] Opening gripper");
      gripper_goal_.position = clam_msgs::ClamGripperCommandGoal::GRIPPER_OPEN;
      clam_gripper_controller_.sendGoal(gripper_goal_);
      clam_gripper_controller_.waitForResult(ros::Duration(10.0)); // has a timeout
      
      // Error check
      if( !clam_gripper_controller_.getState().isDone() ||
          !clam_gripper_controller_.getResult()->reached_goal )
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
      gripper_goal_.position = clam_msgs::ClamGripperCommandGoal::GRIPPER_CLOSE;
      //gripper_goal_.end_effector_setpoint = 0.0; // -0.1
      clam_gripper_controller_.sendGoal(gripper_goal_);
      clam_gripper_controller_.waitForResult(ros::Duration(10.0)); // has a timeout

      // Error check
      if( !clam_gripper_controller_.getState().isDone() ||
          !clam_gripper_controller_.getResult()->reached_goal )
      {
        ROS_ERROR("[gripper test] Timeout: Unable to close end effector");
        return 2;
      }

    }
    ROS_WARN("Sleeping");
    ros::Duration(5).sleep();
    ++i;
  }

  return 0;


  ROS_INFO("[gripper test] Node exiting");
  return 0;
}


