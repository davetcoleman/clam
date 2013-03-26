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
// Desc:   Simple tools for showing parts of a robot in Rviz, such as the gripper or arm

#ifndef BLOCK_GRASP_GENERATOR_ROBOT_VIZ_TOOLS_
#define BLOCK_GRASP_GENERATOR_ROBOT_VIZ_TOOLS_

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>

// ROS
#include <tf_conversions/tf_eigen.h>

// Boost
#include <boost/shared_ptr.hpp>


namespace block_grasp_generator
{

class RobotVizTools
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // A ROS publisher
  ros::Publisher rviz_marker_pub_;

  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Strings
  std::string marker_topic_; // topic to publish to rviz
  std::string ee_group_name_; // end effector group name
  std::string planning_group_name_; // planning group we are working with
  std::string base_link_; // name of base link of robot

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_;

  // End Effector Markers
  bool ee_marker_is_loaded_; // determines if we have loaded the marker or not
  visualization_msgs::MarkerArray marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_;
  std::vector<geometry_msgs::Pose> marker_poses_;

  // Whether to actually publish to rviz or not
  bool muted_;

public:

  /**
   * \brief Constructor
   */
  RobotVizTools(std::string marker_topic, std::string ee_group_name, std::string planning_group_name, std::string base_link, 
                planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor) :
    marker_topic_(marker_topic),
    ee_group_name_(ee_group_name),
    planning_group_name_(planning_group_name),
    base_link_(base_link),
    planning_scene_monitor_(planning_scene_monitor),
    ee_marker_is_loaded_(false),
    marker_lifetime_(ros::Duration(30.0)),
    nh_("~"),
    muted_(false)
  {

    // -----------------------------------------------------------------------------------------------
    // Rviz Visualizations
    rviz_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);
    ros::spinOnce();
    ros::Duration(0.5).sleep(); // TODO: better way of doing this?

    ROS_DEBUG_STREAM_NAMED("robot_viz","Visualizing rviz markers on topic " << marker_topic_);
  }

  /**
   * \brief Move the robot arm to the ik solution in rviz
   * \param joint_values - the in-order list of values to set the robot's joints
   * \return true if it is successful
   */
  bool publishPlanningScene(std::vector<double> joint_values)
  {
    if(muted_)
      return true; // this function will only work if we have loaded the publishers

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing planning scene");

    // Output debug
    //ROS_INFO_STREAM_NAMED("robot_viz","Joint values being sent to planning scene:");
    //std::copy(joint_values.begin(),joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));

    // Update planning scene
    robot_state::JointStateGroup* joint_state_group = planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst()
      .getJointStateGroup(planning_group_name_);
    joint_state_group->setVariableValues(joint_values);

    //    planning_scene_monitor_->updateFrameTransforms();
    planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    return true;
  }

  /**
   * \brief 
   * \return true if it is successful
   */
  // Call this once at begining to load the robot marker
  bool loadEEMarker()
  {
    // -----------------------------------------------------------------------------------------------
    // Get end effector group

    // Create color to use for EE markers
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1.0;
    marker_color.g = 1.0;
    marker_color.b = 1.0;
    marker_color.a = 0.85;

    // Get robot state
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

    // Get joint state group
    robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(ee_group_name_);
    if( joint_state_group == NULL ) // make sure EE_GROUP exists
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","Unable to find joint state group " << ee_group_name_ );
      return false;
    }

    // Get link names that are in end effector
    const std::vector<std::string>
      &ee_link_names = joint_state_group->getJointModelGroup()->getLinkModelNames();
    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of links in group " << ee_group_name_ << ": " << ee_link_names.size());

    // Robot Interaction - finds the end effector associated with a planning group
    robot_interaction::RobotInteraction robot_interaction( planning_scene_monitor_->getRobotModel() );

    // Decide active end effectors
    robot_interaction.decideActiveEndEffectors(planning_group_name_);

    // Get active EE
    std::vector<robot_interaction::RobotInteraction::EndEffector>
      active_eef = robot_interaction.getActiveEndEffectors();
    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of active end effectors: " << active_eef.size());
    if( !active_eef.size() )
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
      return false;
    }

    // Just choose the first end effector TODO: better logic?
    robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

    // -----------------------------------------------------------------------------------------------
    // Get EE link markers for Rviz
    robot_state.getRobotMarkers(marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of rviz markers in end effector: " << marker_array_.markers.size());

    // Change pose from Eigen to TF
    try{
      tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
    }
    catch(...)
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","Didn't find link state for " << eef.parent_link);
    }

    // Offset from gasp_pose to end effector
    static const double X_OFFSET = 0.0; //0.15;

    // Allow a transform from our pose to the end effector position
    grasp_pose_to_eef_pose_.position.x = X_OFFSET;
    grasp_pose_to_eef_pose_.position.y = 0;
    grasp_pose_to_eef_pose_.position.z = 0;
    grasp_pose_to_eef_pose_.orientation.x = 0;
    grasp_pose_to_eef_pose_.orientation.y = 0;
    grasp_pose_to_eef_pose_.orientation.z = 0;
    grasp_pose_to_eef_pose_.orientation.w = 1;

    // Copy original marker poses to a vector
    for (std::size_t i = 0 ; i < marker_array_.markers.size() ; ++i)
    {
      marker_poses_.push_back( marker_array_.markers[i].pose );
    }

    // Record that we have loaded the gripper
    ee_marker_is_loaded_ = true;

    return true;
  }

  /**
   * \brief Publish an end effector to rviz
   * \return true if it is successful
   */
  bool publishEEMarkers(const geometry_msgs::Pose &grasp_pose)
  {
    if(muted_)
      return true;

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing end effector markers");

    //ROS_INFO_STREAM("Mesh (" << grasp_pose.position.x << ","<< grasp_pose.position.y << ","<< grasp_pose.position.z << ")");

    // -----------------------------------------------------------------------------------------------
    // Make sure EE Marker is loaded
    if( !ee_marker_is_loaded_ )
    {
      ROS_INFO_STREAM_NAMED("robot_viz","Loading end effector rviz markers");
      if( !loadEEMarker() )
      {
        ROS_WARN_STREAM_NAMED("robot_viz","Unable to publish EE marker");
        return false;
      }
    }

    // -----------------------------------------------------------------------------------------------
    // Process each link of the end effector
    for (std::size_t i = 0 ; i < marker_array_.markers.size() ; ++i)
    {
      // Header
      marker_array_.markers[i].header.frame_id = base_link_;
      marker_array_.markers[i].header.stamp = ros::Time::now();

      // Options
      marker_array_.markers[i].lifetime = marker_lifetime_;

      // Options for meshes
      if( marker_array_.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE )
      {
        marker_array_.markers[i].mesh_use_embedded_materials = true;
      }

      // -----------------------------------------------------------------------------------------------
      // Do some math for the offset
      // grasp_pose             - our generated grasp
      // markers[i].pose        - an ee link's pose relative to the whole end effector
      // grasp_pose_to_eef_pose_ - the offset from the grasp pose to eef_pose - probably nothing
      tf::Pose tf_root_to_marker;
      tf::Pose tf_root_to_mesh;
      tf::Pose tf_pose_to_eef;

      // Simple conversion from geometry_msgs::Pose to tf::Pose
      tf::poseMsgToTF(grasp_pose, tf_root_to_marker);
      tf::poseMsgToTF(marker_poses_[i], tf_root_to_mesh);
      tf::poseMsgToTF(grasp_pose_to_eef_pose_, tf_pose_to_eef);

      // Conversions
      tf::Pose tf_eef_to_mesh = tf_root_to_link_.inverse() * tf_root_to_mesh;
      tf::Pose tf_marker_to_mesh = tf_pose_to_eef * tf_eef_to_mesh;
      tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_marker_to_mesh;
      tf::poseTFToMsg(tf_root_to_mesh_new, marker_array_.markers[i].pose);
      // -----------------------------------------------------------------------------------------------

      //ROS_INFO_STREAM("Marker " << i << ":\n" << marker_array_.markers[i]);

      rviz_marker_pub_.publish( marker_array_.markers[i] );
      ros::Duration(0.05).sleep();  // Sleep to prevent markers from being 'skipped' in rviz
    }
    
    return true;
  }

  /**
   * \brief Publish an marker of a mesh to rviz
   * \return true if it is successful
   */
  bool publishMesh(double x, double y, double z, double qx, double qy, double qz, double qw )
  {
    if(muted_)
      return true; // this function will only work if we have loaded the publishers

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing mesh");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Mesh";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://clam_description/stl/gripper_base_link.STL";

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Make line color
    std_msgs::ColorRGBA color;
    color.r = 0.8;
    color.g = 0.1;
    color.b = 0.1;
    color.a = 1.0;


    // Point
    geometry_msgs::Point point_a;
    point_a.x = x;
    point_a.y = y;
    point_a.z = z;
    //ROS_INFO_STREAM("Publishing marker \n" << point_a );

    // Add the point pair to the line message
    marker.points.push_back( point_a );
    marker.colors.push_back( color );

    marker.lifetime = marker_lifetime_;

    rviz_marker_pub_.publish( marker );

    return true;
  }

  /**
   * \brief Publish an marker of a sphere to rviz
   * \return true if it is successful
   */
  bool publishSphere(double x, double y, double z)
  {
    if(muted_)
      return true; // this function will only work if we have loaded the publishers

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing sphere");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Sphere";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Make line color
    std_msgs::ColorRGBA color;
    color.r = 0.1;
    color.g = 0.1;
    color.b = 0.8;
    color.a = 1.0;


    // Point
    geometry_msgs::Point point_a;
    point_a.x = x;
    point_a.y = y;
    point_a.z = z;
    //ROS_INFO_STREAM("Publishing marker \n" << point_a );

    // Add the point pair to the line message
    marker.points.push_back( point_a );
    marker.colors.push_back( color );


    rviz_marker_pub_.publish( marker );

    return true;
  }

  /**
   * \brief Publish an marker of an arrow to rviz
   * \return true if it is successful
   */
  bool publishArrow(const geometry_msgs::Pose &pose)
  {
    if(muted_)
      return true;

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing arrow");

    //ROS_INFO_STREAM("Arrow (" << pose.position.x << ","<< pose.position.y << ","<< pose.position.z << ")");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Arrow";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    static int id = 0;
    marker.id = ++id;

    marker.pose = pose;

    marker.scale.x = 0.05; //0.025; // arrow width - but i would call this the length
    marker.scale.y = 0.005; // arrow height
    marker.scale.z = 0.005; // arrow length

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.lifetime = marker_lifetime_;

    rviz_marker_pub_.publish( marker );
    ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

    return true;
  }

  /**
   * \brief Publish an marker of a block to Rviz
   * \return true if it is successful
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const double& block_size, bool isRed)
  {
    if(muted_)
      return true;

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing block");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Block";

    static int id = 0;
    marker.id = ++id;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose
    marker.pose = pose;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set marker size
    marker.scale.x = block_size;
    marker.scale.y = block_size;
    marker.scale.z = block_size;

    // Set marker color
    if(isRed)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    marker.color.a = 0.5;

    marker.lifetime = marker_lifetime_;

    //ROS_INFO_STREAM("Publishing block with pose \n" << marker );
    rviz_marker_pub_.publish( marker );
    ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

    return true;
  }

  /**
   * \brief Set this class to not actually publish anything to Rviz. verbose=false
   */
  void setxMuted(bool muted)
  {
    muted_ = muted;
  }

}; // class

typedef boost::shared_ptr<RobotVizTools> RobotVizToolsPtr;
typedef boost::shared_ptr<const RobotVizTools> RobotVizToolsConstPtr;

} // namespace

#endif
