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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clam_moveit_experimental_node", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string group_name = argc > 1 ? argv[1] : "arm";
  ROS_INFO_STREAM("Preparing to send command to group = " << group_name);

  move_group_interface::MoveGroup group(group_name);
  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  
  ROS_INFO_STREAM("Current\n" << current_pose);

  std::vector<geometry_msgs::Pose> goal_pose(2);
  goal_pose[0] = current_pose.pose;
  goal_pose[1] = goal_pose[0];
  goal_pose[1].position.x -= 0.01;

  ROS_INFO_STREAM("Goal\n" << goal_pose[1]);

  group.followConstraints(goal_pose);
  move_group_interface::MoveGroup::Plan plan;
  group.plan(plan);
  sleep(2);

  ROS_INFO("Node exiting");
  return 0;
}
