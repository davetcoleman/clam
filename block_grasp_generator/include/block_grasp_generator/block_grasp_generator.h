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

// Author: Dave Coleman
//   Desc:   Generates grasps for a cube

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <manipulation_msgs/Grasp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Rviz
//#include "robot_viz_tools.h"
#include <block_grasp_generator/robot_viz_tools.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>


// MoveIt
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace block_grasp_generator
{

static const double RAD2DEG = 57.2957795;

struct RobotGraspData
{
  RobotGraspData() :    
    // Fill in default values where possible:
    base_link_("/base_link"), 
    grasp_depth_(0.12), 
    angle_resolution_(16),
    approach_retreat_desired_dist_(0.6),
    approach_retreat_min_dist_(0.4)
  {}
  sensor_msgs::JointState pre_grasp_posture_;
  sensor_msgs::JointState grasp_posture_;
  std::string base_link_; // name of global frame with z pointing up
  std::string ee_parent_link_; // the last link in the kinematic chain before the end effector, e.g. "/gripper_roll_link"
  double grasp_depth_; // distance from center point of object to end effector
  int angle_resolution_; // generate grasps at PI/angle_resolution increments
  double approach_retreat_desired_dist_;
  double approach_retreat_min_dist_;
};


// Class
class BlockGraspGenerator
{
private:

  // Grasp axis orientation
  enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
  enum grasp_direction_t {UP, DOWN};

  // class for publishing stuff to rviz
  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // Transform from frame of box to global frame
  Eigen::Affine3d block_global_transform_; 

public:

  // Constructor
  BlockGraspGenerator( RobotVizToolsPtr rviz_tools);                  

  // Destructor
  ~BlockGraspGenerator();

  // Create all possible grasp positions for a block
  bool generateGrasps(const geometry_msgs::Pose& block_pose, const RobotGraspData& grasp_data,
                      std::vector<manipulation_msgs::Grasp>& possible_grasps);

private:

  // Create grasp positions in one axis
  bool generateAxisGrasps(std::vector<manipulation_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
                          grasp_direction_t direction, const RobotGraspData& grasp_data);
                          
  // Show all grasps in Rviz
  void visualizeGrasps(const std::vector<manipulation_msgs::Grasp>& possible_grasps,
                       const geometry_msgs::Pose& block_pose);

}; // end of class

typedef boost::shared_ptr<BlockGraspGenerator> BlockGraspGeneratorPtr;
typedef boost::shared_ptr<const BlockGraspGenerator> GraspGeneratorConstPtr;

} // namespace


