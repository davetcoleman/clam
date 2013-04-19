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
   Desc:   Tests the grasp generator filter
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Clam
#include <clam_msgs/ClamGripperCommandAction.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp 
#include <block_grasp_generator/block_grasp_generator.h>
#include <block_grasp_generator/grasp_filter.h>

namespace block_grasp_generator
{

// Static const vars
static const std::string EE_LINK = "gripper_roll_link";
static const std::string EE_PARENT_LINK = "gripper_roll_link";

// Required for RobotVizTools:
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_JOINT = "gripper_finger_joint"; // TODO: remove this dependency!!
static const std::string BASE_LINK = "/base_link";
static const double BLOCK_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

  // class for publishing stuff to rviz
  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // class for filter object
  block_grasp_generator::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests) :
    nh_("~")
  {

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    rviz_tools_.reset(new block_grasp_generator::RobotVizTools(RVIZ_MARKER_TOPIC, EE_GROUP, PLANNING_GROUP_NAME, BASE_LINK));
    rviz_tools_->setLifetime(40.0);
    rviz_tools_->setMuted(false);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    loadRobotGraspData(); // Load robot specific data
    block_grasp_generator_.reset( new block_grasp_generator::BlockGraspGenerator(rviz_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    bool rviz_verbose = true;
    grasp_filter_.reset(new block_grasp_generator::GraspFilter(BASE_LINK, rviz_verbose, rviz_tools_, PLANNING_GROUP_NAME) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random blocks

    geometry_msgs::Pose block_pose;
    std::vector<manipulation_msgs::Grasp> possible_grasps;

    // Loop
    for (int i = 0; i < num_tests; ++i)
    {
      ROS_INFO_STREAM_NAMED("test","Adding random block " << i+1 << " of " << num_tests);

      generateRandomBlock(block_pose);
      //getTestBlock(block_pose);
      rviz_tools_->publishBlock(block_pose, BLOCK_SIZE, false);

      possible_grasps.clear();

      // Generate set of grasps for one block
      rviz_tools_->setMuted(true); // we don't want to see unfiltered grasps
      block_grasp_generator_->generateGrasps( block_pose, grasp_data_, possible_grasps);
      rviz_tools_->setMuted(false);

      // Filter the grasp for only the ones that are reachable
      grasp_filter_->filterGrasps(possible_grasps);

      // Visualize them
      block_grasp_generator_->visualizeGrasps(possible_grasps, block_pose);

      // Make sure ros is still going
      if(!ros::ok)
        break;
    }


  }

  void loadRobotGraspData()
  {
    // -------------------------------
    // Create pre-grasp posture
    grasp_data_.pre_grasp_posture_.header.frame_id = BASE_LINK;
    grasp_data_.pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    grasp_data_.pre_grasp_posture_.name.resize(1);
    grasp_data_.pre_grasp_posture_.name[0] = EE_JOINT;
    // Position of joints
    grasp_data_.pre_grasp_posture_.position.resize(1);
    grasp_data_.pre_grasp_posture_.position[0] = clam_msgs::ClamGripperCommandGoal::GRIPPER_OPEN;

    // -------------------------------
    // Create grasp posture
    grasp_data_.grasp_posture_.header.frame_id = BASE_LINK;
    grasp_data_.grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    grasp_data_.grasp_posture_.name.resize(1);
    grasp_data_.grasp_posture_.name[0] = EE_JOINT;
    // Position of joints
    grasp_data_.grasp_posture_.position.resize(1);
    grasp_data_.grasp_posture_.position[0] = clam_msgs::ClamGripperCommandGoal::GRIPPER_CLOSE;

    // -------------------------------
    // Links
    grasp_data_.base_link_ = BASE_LINK;
    grasp_data_.ee_parent_link_ = EE_PARENT_LINK;

    // -------------------------------
    // Nums
    grasp_data_.approach_retreat_desired_dist_ = 0.05;
    grasp_data_.approach_retreat_min_dist_ = 0.025;
    grasp_data_.grasp_depth_ = 0.16;
  }

  void getTestBlock(geometry_msgs::Pose& block_pose)
  {
    // Position
    geometry_msgs::Pose start_block_pose;
    geometry_msgs::Pose end_block_pose;

    start_block_pose.position.x = 0.2;
    start_block_pose.position.y = 0.0;
    start_block_pose.position.z = 0.02;

    end_block_pose.position.x = 0.25;
    end_block_pose.position.y = 0.15;
    end_block_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block_pose.orientation.x = quat.x();
    start_block_pose.orientation.y = quat.y();
    start_block_pose.orientation.z = quat.z();
    start_block_pose.orientation.w = quat.w();

    angle = M_PI / 1.1;
    quat = Eigen::Quaterniond(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    end_block_pose.orientation.x = quat.x();
    end_block_pose.orientation.y = quat.y();
    end_block_pose.orientation.z = quat.z();
    end_block_pose.orientation.w = quat.w();

    // Choose which block to test
    block_pose = start_block_pose;
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

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 100;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(5);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  block_grasp_generator::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
