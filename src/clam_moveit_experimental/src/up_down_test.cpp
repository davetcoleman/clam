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
   Desc:   Testing moving the arm from sleep to default pose
*/

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// ClamArm
#include <clam_msgs/ClamArmAction.h> // for controlling the gripper

namespace clam_moveit_experimental
{

// *********************************************************************************************************
// *********************************************************************************************************
// Up down test class
// *********************************************************************************************************
// *********************************************************************************************************

class UpDownTest
{
private:

  // A ROS publisher
  ros::Publisher marker_pub_;

  // A shared node handle
  ros::NodeHandle nh_;

public:
  // *********************************************************************************************************
  // Constructor
  // *********************************************************************************************************
  UpDownTest()
  {
    // -----------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_("clam_arm", true);
    clam_msgs::ClamArmGoal clam_arm_goal_; // sent to the clam_arm_client_server

    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO("[up down test] Waiting for the clam_arm action server");
    }


    // -----------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO("[up down test] Closing gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    clam_arm_client_.waitForResult(ros::Duration(10.0)); // has a timeout
    if( !clam_arm_client_.getState().isDone() || !clam_arm_client_.getResult()->success )
    {
      ROS_ERROR("[up down test] Timeout: Unable to close end effector");
    }

    while(true)
    {
      // -----------------------------------------------------------------------------------------------
      // Go to home position
      ROS_INFO("[up down test] Sending arm to home position");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      clam_arm_client_.waitForResult(ros::Duration(20.0));
      if( !clam_arm_client_.getState().isDone() || !clam_arm_client_.getResult()->success )
      {
        ROS_ERROR("[up down test] Timeout: Unable to move to home position");
      }
      ros::Duration(5.0);

      // -----------------------------------------------------------------------------------------------
      // Go to sleep
      ROS_INFO("[up down test] Sending arm to shutdown position");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::SHUTDOWN;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      clam_arm_client_.waitForResult(ros::Duration(20.0));
      if( !clam_arm_client_.getState().isDone() || !clam_arm_client_.getResult()->success )
      {
        ROS_ERROR("[up down test] Timeout: Unable to move to shutdown position");
      }
      ros::Duration(5.0);
    }


    ROS_INFO("[up down test] Node exiting");
  }

  // *********************************************************************************************************
  // Deconstructor
  // *********************************************************************************************************
  ~UpDownTest()
  {

  }

}; // end of class

} // namespace

// Simple test program
int main(int argc, char **argv)
{
  // -----------------------------------------------------------------------------------------------
  // Initialize node
  ros::init(argc, argv, "clam_up_down_test", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  clam_moveit_experimental::UpDownTest test;

  return 0;
}
