/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CU Boulder
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

/**
 * \brief   Simple pick place for blocks using ClamArm
 * \author  Dave Coleman
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Clam
#include <clam_msgs/ClamGripperCommandAction.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "/base_link";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_JOINT = "gripper_finger_joint";
static const std::string EE_PARENT_LINK = "gripper_roll_link";
static const std::string BLOCK_NAME = "block";
static const double BLOCK_SIZE = 0.04;

// class for publishing stuff to rviz
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

// grasp generator
moveit_simple_grasps::SimpleGraspsPtr moveit_simple_grasps_;

// data for generating grasps
moveit_simple_grasps::GraspData grasp_data_;

// our interface with MoveIt
boost::scoped_ptr<move_group_interface::MoveGroup> group_;

// block description
typedef std::pair<std::string,geometry_msgs::Pose> MetaBlock;


double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void generateRandomBlock(geometry_msgs::Pose& block_pose)
{
  // Position
  block_pose.position.x = fRand(0.1,0.45);
  //  block_pose.position.y = fRand(-0.25,0.25);
  block_pose.position.y = fRand(-0.25,-0.05);
  block_pose.position.z = 0.02;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();
}

/*
void publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name)
{
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;
  co.id = block_name;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BLOCK_SIZE;
  co.primitive_poses.resize(1);
  co.primitive_poses[0] = block_pose;
  pub_co_.publish(co);
}
*/

bool pick(const geometry_msgs::Pose& block_pose, std::string block_name)
{
  ROS_WARN_STREAM_NAMED("","picking block "<< block_name);

  std::vector<moveit_msgs::Grasp> possible_grasps;

  // Pick grasp
  moveit_simple_grasps_->generateBlockGrasps( block_pose, grasp_data_, possible_grasps );

  // Visualize them
  visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
  //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

  // Prevent collision with table
  group_->setSupportSurfaceName("tabletop_link");

  //ROS_WARN_STREAM_NAMED("","testing grasp 1:\n" << possible_grasps[0]);

  //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << possible_grasps[0]);
  //ROS_INFO_STREAM_NAMED("","\n\n\nGrasp 10\n" << possible_grasps[10]);

  return group_->pick(block_name, possible_grasps);
}

bool place(const MetaBlock block)
{
  ROS_WARN_STREAM_NAMED("","placing block "<< block.first);

  std::vector<moveit_msgs::PlaceLocation> place_locations;
  std::vector<moveit_msgs::Grasp> possible_grasps;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = BASE_LINK;
  pose_stamped.header.stamp = ros::Time::now();

  // Generate grasps
  moveit_simple_grasps_->generateBlockGrasps( block.second, grasp_data_, possible_grasps );

  // Visualize them
  visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
  //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

  // Convert 'grasps' to place_locations format
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    // Create new place location
    moveit_msgs::PlaceLocation place_loc;

    // Pose
    pose_stamped.pose = possible_grasps[i].grasp_pose.pose;
    place_loc.place_pose = pose_stamped;

    // Publish to Rviz
    visual_tools_->publishArrow(pose_stamped.pose);

    // Approach & Retreat
    place_loc.pre_place_approach = possible_grasps[i].pre_grasp_approach;
    //ROS_WARN_STREAM_NAMED("","is the same? \n" << place_loc.approach);
    place_loc.post_place_retreat = possible_grasps[i].post_grasp_retreat;

    // Post place posture - use same as pre-grasp posture (the OPEN command_
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  }
  ROS_INFO_STREAM_NAMED("pick_place","Created " << place_locations.size() << " place locations");

  // Prevent collision with table
  group_->setSupportSurfaceName("tabletop_link");

  group_->setPlannerId("RRTConnectkConfigDefault");

  return group_->place(block.first, place_locations);
}

void getGoalBlocks(std::vector<MetaBlock>& block_locations)
{
  // Position
  geometry_msgs::Pose block_pose;
  block_pose.position.x = 0.3;
  block_pose.position.y = 0.2;
  block_pose.position.z = BLOCK_SIZE/2 * 0.9;

  // Orientation
  double angle = M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();

  // Block1
  MetaBlock block1 = MetaBlock("Block1", block_pose);
  block_locations.push_back(block1);

  // Block2
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block2 = MetaBlock("Block2", block_pose);
  block_locations.push_back(block2);

  // Block3
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block3 = MetaBlock("Block3", block_pose);
  block_locations.push_back(block3);

  // Block4
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block4 = MetaBlock("Block4", block_pose);
  block_locations.push_back(block3);
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("main","Starting clam pick place");
  ros::init (argc, argv, "clamarm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ---------------------------------------------------------------------------------------------
  // Load grasp data specific to our robot
  ros::NodeHandle nh("~");
  if(!grasp_data_.loadRobotGraspData(nh, EE_GROUP))
  {
      ROS_ERROR_STREAM_NAMED("simple_pick_place", "Cannot load end_effector data");
      return 1;
  }

  // ---------------------------------------------------------------------------------------------
  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools( BASE_LINK, RVIZ_MARKER_TOPIC));
  visual_tools_->loadEEMarker(grasp_data_.ee_group_, PLANNING_GROUP_NAME);

  // ---------------------------------------------------------------------------------------------
  // Load grasp generator
  moveit_simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

  // ---------------------------------------------------------------------------------------------
  // Create MoveGroup
  group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
  group_->setPlanningTime(30.0);

  ros::Duration(0.5).sleep();

  // --------------------------------------------------------------------------------------------------------
  // Start pick and place loop

  geometry_msgs::Pose start_block_pose;
  geometry_msgs::Pose end_block_pose;

  std::vector<MetaBlock> goal_block_locations;
  getGoalBlocks(goal_block_locations);

  // Remvoed attached objects
  visual_tools_->removeAllCollisionObjects();

  int goal_id = 0;
  while(true && ros::ok())
  {
    bool foundBlock = false;
    while(!foundBlock && ros::ok())
    {
      generateRandomBlock(start_block_pose);

      visual_tools_->publishCollisionBlock(start_block_pose, goal_block_locations[goal_id].first, BLOCK_SIZE);

      ROS_INFO_STREAM_NAMED("simple_pick_place","Published collision object");

      //MetaBlock start_block = MetaBlock(goal_block_locations[goal_id].first,start_block_pose);

      if( !pick(start_block_pose, goal_block_locations[goal_id].first) )
      {
        ROS_ERROR_STREAM_NAMED("simple_pick_place","Pick failed. Retrying.");
        visual_tools_->cleanupCO(goal_block_locations[goal_id].first);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick");
        foundBlock = true;
      }
    }

    ros::Duration(1.0).sleep();

    bool putBlock = false;
    while(!putBlock && ros::ok())
    {
      //generateRandomBlock(end_block_pose);
      end_block_pose = goal_block_locations[goal_id].second;

      if( !place(goal_block_locations[goal_id]) )
      {
        ROS_ERROR_STREAM_NAMED("simple_pick_place","Place failed. Retrying.");
      }
      else
      {
        ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
        putBlock = true;
        ++goal_id;
        if( goal_id > goal_block_locations.size() )
        {
          ROS_WARN_STREAM_NAMED("","Out of goal locations");
          ros::shutdown();
          return 0;
        }
      }
    }

    // Cycle placed block to become pick block
    start_block_pose = end_block_pose;
  }

  ros::shutdown();
  return 0;
}
