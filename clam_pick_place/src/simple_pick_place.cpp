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

// Clam
#include <clam_msgs/ClamGripperCommandAction.h>

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
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_JOINT = "gripper_finger_joint";
static const std::string EE_PARENT_LINK = "gripper_roll_link";
static const std::string BLOCK_NAME = "block";
static const double BLOCK_SIZE = 0.04;

// Planning Scene Monitor
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

// class for publishing stuff to rviz
block_grasp_generator::RobotVizToolsPtr rviz_tools_;

// grasp generator
block_grasp_generator::GraspGeneratorPtr grasp_generator_;

// publishers
ros::Publisher pub_co_;
ros::Publisher pub_aco_;

struct PickPlaceData
{
  sensor_msgs::JointState pre_grasp_posture_;
  sensor_msgs::JointState grasp_posture_;
};

PickPlaceData data_;

void generateRobotSpecificData()
{
  // -------------------------------
  // Create pre-grasp posture
  pre_grasp_posture_.header.frame_id = BASE_LINK;
  pre_grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  pre_grasp_posture_.name.resize(1, EE_JOINT);
  //pre_grasp_posture_.name.resize(1);
  //pre_grasp_posture_.name[0] = EE_JOINT;
  // Position of joints
  pre_grasp_posture_.position.resize(1);
  pre_grasp_posture_.position[0] = clam_msgs::ClamGripperCommandGoal::GRIPPER_OPEN;

  // -------------------------------
  // Create grasp posture
  grasp_posture_.header.frame_id = BASE_LINK;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.name.resize(1, EE_JOINT);
  //grasp_posture_name.resize(1);
  //grasp_posture.name[0] = EE_JOINT;
  // Position of joints
  grasp_posture_.position.resize(1);
  grasp_posture_.position[0] = clam_msgs::ClamGripperCommandGoal::GRIPPER_CLOSE;

  // -------------------------------


  // -------------------------------
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void generateRandomBlock(geometry_msgs::Pose& block_pose)
{
  // Position
  block_pose.position.x = fRand(0.1,0.45);
  block_pose.position.y = fRand(-0.25,0.25);
  block_pose.position.z = 0.02;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();
}

bool pick(move_group_interface::MoveGroup &group, const geometry_msgs::Pose& block_pose)
{
  std::vector<manipulation_msgs::Grasp> grasps;

  // Pick grasp
  bool dual_approach = true; // approach straight down and also from an angle
  grasp_generator_->generateGrasps( block_pose, grasps, pre_grasp_posture_, grasp_posture_, dual_approach );

  // Prevent collision with table
  group.setSupportSurfaceName("tabletop_link");

  return group.pick(BLOCK_NAME, grasps);
}

bool place(move_group_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::PlaceLocation> loc;

  std::vector<manipulation_msgs::Grasp> grasps;
  sensor_msgs::JointState pre_grasp_posture;
  sensor_msgs::JointState grasp_posture;

  // Create random block pose
  geometry_msgs::Pose block_pose;
  generateRandomBlock(block_pose);

  bool dual_approach = false; // approach straight down and also from an angle
  grasp_generator_->generateGrasps( block_pose, grasps, pre_grasp_posture, grasp_posture, dual_approach );

  for (std::size_t i = 0; i < grasps.size(); ++i)
  {
    ROS_INFO_STREAM_NAMED("pose #",i);

    // Create new place location
    manipulation_msgs::PlaceLocation place_loc;

    // Pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = BASE_LINK;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = grasps[i].grasp_pose.pose;
    place_loc.place_pose = pose_stamped;

    // Publish to Rviz
    rviz_tools_->publishArrow(pose_stamped.pose);

    // Approach  TODO - get from grasp generator
    place_loc.approach.direction.header.frame_id = BASE_LINK;
    place_loc.approach.direction.header.stamp = ros::Time::now();
    place_loc.approach.direction.vector.z = -1.0;
    place_loc.approach.min_distance = 0.025;
    place_loc.approach.desired_distance = 0.050;

    // Retreat
    place_loc.retreat.direction.header.frame_id = BASE_LINK;
    place_loc.retreat.direction.header.stamp = ros::Time::now();
    place_loc.retreat.direction.vector.z = 1.0;
    place_loc.retreat.min_distance = 0.025;
    place_loc.retreat.desired_distance = 0.050;

    // Post place posture
    place_loc.post_place_posture.name.resize(1, EE_JOINT);
    place_loc.post_place_posture.position.resize(1);
    place_loc.post_place_posture.position[0] = clam_msgs::ClamGripperCommandGoal::GRIPPER_OPEN;

    loc.push_back(place_loc);
  }

  // Prevent collision with table
  group.setSupportSurfaceName("tabletop_link");

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

  return group.place(BLOCK_NAME, loc);
}

void cleanupBlocks()
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = BASE_LINK;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  aco.link_name = EE_PARENT_LINK;
  ros::WallDuration(0.5).sleep();
  pub_aco_.publish(aco);
  ros::WallDuration(0.5).sleep();
  pub_aco_.publish(aco);

  // Clean up old collision objects
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;
  co.id = BLOCK_NAME;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  ros::WallDuration(0.5).sleep();
  pub_co_.publish(co);
  ros::WallDuration(0.5).sleep();
  pub_co_.publish(co);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "clamarm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  pub_co_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco_ = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);

  ros::Duration(1.0).sleep();

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
  // Load the Robot Viz Tools for publishing to Rviz
  rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP, PLANNING_GROUP_NAME,
                                                              BASE_LINK, planning_scene_monitor_ ));

  // ---------------------------------------------------------------------------------------------
  // Load grasp generator
  grasp_generator_.reset(new block_grasp_generator::GraspGenerator( BASE_LINK, rviz_tools_ ));

  // ---------------------------------------------------------------------------------------------
  // Create MoveGroup
  move_group_interface::MoveGroup group(PLANNING_GROUP_NAME);
  group.setPlanningTime(30.0);

  ros::Duration(1.0).sleep();
  /*
  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co);

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
  pub_co_.publish(co);
  */

  // Cleanup old blocks
  cleanupBlocks();

  // Create a block pose
  geometry_msgs::Pose start_block_pose;
  generateRandomBlock(start_block_pose);

  // Add the block
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;
  co.id = BLOCK_NAME;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BLOCK_SIZE;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BLOCK_SIZE;
  co.primitive_poses.resize(1);
  co.primitive_poses[0] = start_block_pose;
  pub_co_.publish(co);
  ROS_INFO_STREAM_NAMED("simple_pick_place","Published collision object");

  // --------------------------------------------------------------------------------------------------------
  // Start pick and place loop

  for (std::size_t i = 0; i < 3; ++i)
  {

    if( !pick(group, start_block_pose) )
    {
      ROS_ERROR_STREAM_NAMED("simple_pick_place","Pick failed");
      break;
    }

    ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick");

    // Show the end block pose
    ros::WallDuration(1.0).sleep();

    if( !place(group) )
    {
      ROS_ERROR_STREAM_NAMED("simple_pick_place","Place failed");
      break;
    }

    ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
  }

  ros::shutdown();
  return 0;
}
