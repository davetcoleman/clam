/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
   Desc:   Simple test of the clam gripper controller
*/

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// ClamArm
#include <clam_msgs/ClamGripperCommandAction.h>


namespace test_clam_gripper_controller
{

// Static const vars
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_LINK = "gripper_roll_link";

// Required for RobotVizTools:
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_JOINT = "gripper_finger_joint"; // TODO: remove this dependency!!
static const std::string BASE_LINK = "/base_link";

class ClamGripperControllerTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_;

  // Action messages
  //  clam_msgs::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server
  //  clam_msgs::PickPlaceFeedback     feedback_;
  //  clam_msgs::PickPlaceResult       result_;

public:

  // Constructor
  ClamGripperControllerTest(int num_tests) :
    nh_("~"),
    clam_arm_client_("clam_arm", true)
  {
    
    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick place","Waiting for the clam_arm action server");
    }


    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO_STREAM_NAMED("pick_place","Opening gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();


    // ---------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO_STREAM_NAMED("pick_place","Opening gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 1;

  ros::init(argc, argv, "test_clam_gripper_controller");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(5);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  test_clam_gripper_controller::ClamGripperControllerTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  return 0;
}
