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

#include <block_grasp_generator/grasp_generator.h>

namespace block_grasp_generator
{

// Constructor
GraspGenerator::GraspGenerator( const std::string base_link,
                                RobotVizToolsPtr rviz_tools) :
  base_link_(base_link),
  rviz_tools_(rviz_tools)
{
  ROS_DEBUG_STREAM_NAMED("grasp","GraspGenerator ready.");
}

// Deconstructor
GraspGenerator::~GraspGenerator()
{
  ROS_DEBUG_STREAM_NAMED("grasp","Unloading GraspGenerator");
}


// Create all possible grasp positions for a block
bool GraspGenerator::generateGrasps(const geometry_msgs::Pose& block_pose, std::vector<manipulation_msgs::Grasp>& possible_grasps,
                                    const sensor_msgs::JointState& pre_grasp_posture, const sensor_msgs::JointState& grasp_posture,
                                    bool dual_approach)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the block's frame (center of block) to /base_link
  tf::poseMsgToEigen(block_pose, transform_);

  // ---------------------------------------------------------------------------------------------
  // Show block
  //rviz_tools_->publishBlock(block_pose, BLOCK_SIZE, true);

  // ---------------------------------------------------------------------------------------------
  // Calculate grasps in two axis in both directions
  generateAxisGrasps( possible_grasps, X_AXIS, DOWN, pre_grasp_posture, grasp_posture, dual_approach );
  generateAxisGrasps( possible_grasps, X_AXIS, UP, pre_grasp_posture, grasp_posture, dual_approach );
  generateAxisGrasps( possible_grasps, Y_AXIS, DOWN, pre_grasp_posture, grasp_posture, dual_approach );
  generateAxisGrasps( possible_grasps, Y_AXIS, UP, pre_grasp_posture, grasp_posture, dual_approach );
  ROS_DEBUG_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  // Visualize results
  visualizeGrasps(possible_grasps, block_pose);

  return true;
}

// Create grasp positions in one axis
bool GraspGenerator::generateAxisGrasps(std::vector<manipulation_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
                                        grasp_direction_t direction, const sensor_msgs::JointState& pre_grasp_posture, 
                                        const sensor_msgs::JointState& grasp_posture, bool dual_approach)
{
  // ---------------------------------------------------------------------------------------------
  // Grasp parameters

  // Create re-usable approach motion
  manipulation_msgs::GripperTranslation gripper_approach;
  gripper_approach.direction.header.stamp = ros::Time::now();
  gripper_approach.desired_distance = .050; // The distance the origin of a robot link needs to travel
  gripper_approach.min_distance = .025; // half of the desired? Untested.

  // Create re-usable retreat motion
  manipulation_msgs::GripperTranslation gripper_retreat;
  gripper_retreat.direction.header.stamp = ros::Time::now();
  gripper_retreat.desired_distance = .050; // The distance the origin of a robot link needs to travel
  gripper_retreat.min_distance = .025; // half of the desired? Untested.

  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = base_link_;

  // ---------------------------------------------------------------------------------------------
  // Variables needed for calculations
  double radius = 0.12; //0.15
  double xb;
  double yb = 0.0; // stay in the y plane of the block
  double zb;
  double angle_resolution = 16.0; // 16.0;
  double theta1 = 0.0; // Where the point is located around the block
  double theta2 = 0.0; // UP 'direction'

  // Gripper direction (UP/DOWN) rotation. UP set by default
  if( direction == DOWN )
  {
    theta2 = M_PI;
  }

  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the block, then later convert it to the base link
   */
  for(int i = 0; i <= angle_resolution; ++i)
  {
    //ROS_DEBUG_STREAM_NAMED("grasp", "Generating grasp " << i );

    // Calculate grasp
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
    case X_AXIS:
      grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

      grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

      break;
    case Y_AXIS:
      grasp_pose =
        Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

      grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

      break;
    case Z_AXIS:
      ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
      return false;

      break;
    }

    // Calculate the theta1 for next time
    theta1 += M_PI / angle_resolution;

    // ---------------------------------------------------------------------------------------------
    // Create a Grasp message
    manipulation_msgs::Grasp new_grasp;

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

    // PreGrasp and Grasp Postures --------------------------------------------------------------------------

    // The internal posture of the hand for the pre-grasp only positions are used
    //sensor_msgs/JointState pre_grasp_posture
    new_grasp.pre_grasp_posture = pre_grasp_posture;

    // The internal posture of the hand for the grasp positions and efforts are used
    //sensor_msgs/JointState grasp_posture
    new_grasp.grasp_posture = grasp_posture;

    // Grasp ------------------------------------------------------------------------------------------------

    // Convert pose to global frame (base_link)
    tf::poseEigenToMsg(transform_ * grasp_pose, grasp_pose_msg.pose);

    // The position of the end-effector for the grasp relative to a reference frame
    // (that is always specified elsewhere, not in this message)
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // The estimated probability of success for this grasp, or some other
    // measure of how "good" it is.
    new_grasp.grasp_quality = 1;

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // an optional list of obstacles that we have semantic information about
    // and that can be touched/pushed/moved in the course of grasping
    //string[] allowed_touch_objects

    // -------------------------------------------------------------------------------------------------------
    // Approach and retreat - add pose twice to possible grasps - two different approach and retreat motions

    if( dual_approach ) // optionally don't do this
    {
      // Straight down ---------------------------------------------------------------------------------------

      // Approach
      gripper_approach.direction.header.frame_id = base_link_;
      gripper_approach.direction.vector.x = 0;
      gripper_approach.direction.vector.y = 0;
      gripper_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      new_grasp.approach = gripper_approach;

      // Retreat
      gripper_retreat.direction.header.frame_id = base_link_;
      gripper_retreat.direction.vector.x = 0;
      gripper_retreat.direction.vector.y = 0;
      gripper_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      new_grasp.retreat = gripper_retreat;

      // Add to vector
      possible_grasps.push_back(new_grasp);
    }

    // Angled with pose -------------------------------------------------------------------------------------

    // Approach
    gripper_approach.direction.header.frame_id = "/gripper_roll_link";
    gripper_approach.direction.vector.x = 1;
    gripper_approach.direction.vector.y = 0;
    gripper_approach.direction.vector.z = 0; // Approach direction (negative z axis)
    new_grasp.approach = gripper_approach;

    // Retreat
    gripper_retreat.direction.header.frame_id = "/gripper_roll_link";
    gripper_retreat.direction.vector.x = -1;
    gripper_retreat.direction.vector.y = 0;
    gripper_retreat.direction.vector.z = 0; // Retreat direction (pos z axis)
    new_grasp.retreat = gripper_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);
  }


  return true;
}

// Show all grasps in Rviz
void GraspGenerator::visualizeGrasps(const std::vector<manipulation_msgs::Grasp>& possible_grasps,
                                     const geometry_msgs::Pose& block_pose)
{
  if(rviz_tools_->isMuted())
    return; // this function will only work if we have loaded the publishers

  // isRed = true if possible_blocks is empty
  //rviz_tools_->publishBlock(block_pose, BLOCK_SIZE, possible_grasps.empty() );
  rviz_tools_->publishSphere(block_pose);

  int i = 0;
  for(std::vector<manipulation_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
      grasp_it < possible_grasps.end(); ++grasp_it)
  {
    geometry_msgs::Pose grasp_pose = grasp_it->grasp_pose.pose;

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose");

    //rviz_tools_->publishSphere(grasp_pose);

    if( i % 2)  // do every other arrow
      rviz_tools_->publishArrow(grasp_pose);
    ++i;

    //rviz_tools_->publishEEMarkers(grasp_pose);

    // Show robot joint positions if available
    //if( !grasp_it->grasp_posture.position.empty() )
    //rviz_tools_->publishPlanningScene(grasp_it->grasp_posture.position);

    ros::Duration(0.005).sleep();
    //ros::Duration(0.1).sleep();
  }
}

} // namespace
