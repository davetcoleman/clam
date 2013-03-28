/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

// Grasp generation
#include <block_grasp_generator/grasp_generator.h>
#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing grasps

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string BASE_LINK = "/base_link";
static const std::string EE_GROUP = "gripper_group"; // TODO: remove this dependency!!
static const std::string EE_JOINT = "gripper_finger_joint"; // TODO: remove this dependency!!
static const std::string BLOCK_NAME = "block";
static const double BLOCK_SIZE = 0.04;

// Planning Scene Monitor
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

// class for publishing stuff to rviz
block_grasp_generator::RobotVizToolsPtr rviz_tools_;

void pick(move_group_interface::MoveGroup &group, const geometry_msgs::Pose& block_pose )
{
  std::vector<manipulation_msgs::Grasp> grasps;

  // Load the Robot Viz Tools for publishing to Rviz
  rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP, PLANNING_GROUP_NAME,
                                                              BASE_LINK, planning_scene_monitor_ ));

  // Create pre-grasp posture
  sensor_msgs::JointState pre_grasp_posture;
  {
    pre_grasp_posture.header.frame_id = BASE_LINK;
    // Name of joints:
    pre_grasp_posture.name.resize(1);
    pre_grasp_posture.name[0] = EE_JOINT;
    // Position of joints
    pre_grasp_posture.position.resize(1);
    pre_grasp_posture.position[0] = 0.0; // OPEN
  }

  // Create grasp posture
  sensor_msgs::JointState grasp_posture;
  {
    grasp_posture.header.frame_id = BASE_LINK;
    // Name of joints:
    grasp_posture.name.resize(1);
    grasp_posture.name[0] = EE_JOINT;
    // Position of joints
    grasp_posture.position.resize(1);
    grasp_posture.position[0] = 1.0; // CLOSE
  }

  // Load grasp generator
  block_grasp_generator::GraspGenerator grasp_generator( BASE_LINK, rviz_tools_ );

  // Pick grasp
  grasp_generator.generateGrasps( block_pose, grasps, pre_grasp_posture, grasp_posture );

  group.setSupportSurfaceName("tabletop_link");
  group.pick(BLOCK_NAME, grasps);
}

void place(move_group_interface::MoveGroup &group, const geometry_msgs::Pose& block_pose)
{
  std::vector<manipulation_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = BASE_LINK;
  pose_stamped.pose = block_pose;

  manipulation_msgs::PlaceLocation place_loc;
  place_loc.place_pose = pose_stamped;

  // Approach
  place_loc.approach.direction.header.frame_id = BASE_LINK;
  place_loc.approach.direction.vector.z = -1.0;
  place_loc.approach.min_distance = 0.025;
  place_loc.approach.desired_distance = 0.050;

  // Retreat
  place_loc.retreat.direction.header.frame_id = BASE_LINK;
  place_loc.retreat.direction.vector.z = 1.0;
  place_loc.retreat.min_distance = 0.025;
  place_loc.retreat.desired_distance = 0.050;

  // Post place posture
  place_loc.post_place_posture.name.resize(1, "r_gripper_joint");
  place_loc.post_place_posture.position.resize(1);
  place_loc.post_place_posture.position[0] = 1;

  loc.push_back(place_loc);
  //group.setSupportSurfaceName("table");


  // add path constraints
  /*
    moveit_msgs::Constraints constr;
    constr.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = p.header.frame_id;
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    group.setPathConstraints(constr);
  */
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place(BLOCK_NAME, loc);
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void generateRandomBlock(geometry_msgs::Pose& block_pose)
{
  // Position
  block_pose.position.x = fRand(0.1,0.55);
  block_pose.position.y = fRand(-0.28,0.28);
  block_pose.position.z = 0.02;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "clamarm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::WallDuration(1.0).sleep();

  // ---------------------------------------------------------------------------------------------
  // Create planning scene monitor
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  if (planning_scene_monitor_->getPlanningScene())
  {
    //planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
  }
  else
  {
    ROS_FATAL_STREAM_NAMED("pick_place_moveit","Planning scene not configured");
  }

  // ---------------------------------------------------------------------------------------------
  // Create MoveGroup
  move_group_interface::MoveGroup group(PLANNING_GROUP_NAME);
  group.setPlanningTime(30.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;

  /*
  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);
  */

  /*
  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.175;
  pub_co.publish(co);
  */

  // Create a block pose
  geometry_msgs::Pose start_block_pose;
  generateRandomBlock(start_block_pose);
  geometry_msgs::Pose end_block_pose;
  generateRandomBlock(end_block_pose);

  // Create a obstacle
  co.id = BLOCK_NAME;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  ros::WallDuration(0.5).sleep();
  pub_co.publish(co);
  ros::WallDuration(0.5).sleep();
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BLOCK_SIZE;
  co.primitive_poses.resize(1);
  co.primitive_poses[0] = start_block_pose;
  pub_co.publish(co);
  ROS_INFO_STREAM_NAMED("simple_pick_place","Published collision object");

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group, start_block_pose);

  ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick");
  ros::WallDuration(1.0).sleep();

  //place(group, end_block_pose);
  //ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
  
  ros::shutdown();
  //ros::waitForShutdown();
  return 0;
}
