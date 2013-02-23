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
   Desc:   Generates grasps
*/

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_interaction/robot_interaction.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace clam_block_manipulation
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_LINK = "gripper_roll_link";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_NAME = "end_effector";
static const std::string GROUP_NAME = "arm";

// Class
class GraspGeneratorServer
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // A ROS publisher
  ros::Publisher marker_pub_;

  // Action Servers and Clients
  /*  actionlib::SimpleActionServer<clam_msgs::PickPlaceAction> action_server_;

  // Action messages
  clam_msgs::PickPlaceFeedback     feedback_;
  clam_msgs::PickPlaceResult       result_;
  clam_msgs::PickPlaceGoalConstPtr goal_;
  */

  // MoveIt Components
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Parameters from goal
  std::string base_link;
  std::string block_link;

public:

  // Constructor
  GraspGeneratorServer(const std::string name) //:
  //    action_server_(name, false),
  {
    base_link = "base_link";
    block_link = "block_link";

    // -----------------------------------------------------------------------------------------------
    // Rviz Visualizations
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("end_effector_marker", 1);


    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    tf_.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_));

    // ---------------------------------------------------------------------------------------------
    // Check planning scene monitor
    if (planning_scene_monitor_->getPlanningScene() && planning_scene_monitor_->getPlanningScene()->isConfigured())
    {
      /*
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
        planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      */
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Planning scene not configured");
    }

    generateGrasps();

    // ---------------------------------------------------------------------------------------------
    // Register the goal and preempt callbacks
    /*    action_server_.registerGoalCallback(boost::bind(&GraspGeneratorServer::goalCB, this));
          action_server_.registerPreemptCallback(boost::bind(&GraspGeneratorServer::preemptCB, this));
          action_server_.start();
    */

    // Announce state
    //    ROS_INFO_STREAM_NAMED("pick_place", "Server ready.");
    //    ROS_INFO_STREAM_NAMED("pick_place", "Waiting for pick command...");
  }

  // Action server sends goals here
  void goalCB()
  {
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Preempted");
    //    action_server_.setPreempted();
  }


  void generateGrasps()
  {
    float angle = M_PI / 1.5;

    // Test pose
    geometry_msgs::Pose block_pose;
    block_pose.position.x = 0.4;
    block_pose.position.y = 0.0;
    block_pose.position.z = 0.02;

    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(float(angle), Eigen::Vector3f(0,0,1)));
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    // TF
    tf::TransformBroadcaster br;
    tf::Transform transform;
    // transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    // transform.setRotation( tf::Quaternion(0, 0, 0) );
    tf::Vector3 origin = tf::Vector3( block_pose.position.x, block_pose.position.y, block_pose.position.z );
    tf::Quaternion rotation = tf::Quaternion( block_pose.orientation.x,block_pose.orientation.y,
                                              block_pose.orientation.z,block_pose.orientation.w );
    transform.setOrigin( origin );
    transform.setRotation( rotation );

    double r = 0.2;
    double xb;
    double yb = 0; // stay in the y plane of the block
    double zb;
    double theta1 = 0;
    double theta2;
    double angle_resolution = 8;

    geometry_msgs::Pose grasp_pose;
    static const double RAD2DEG = 57.2957795;

    ros::Rate rate(0.5);
    while (nh_.ok())
    {
      // Send Transform
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link, block_link));

      // Calculate grasp
      xb = r*cos(theta1);
      zb = r*sin(theta1);

      theta2 = 3*M_PI/2; // straight up
      theta2 = 0; // left
      theta2 = theta1 + 0.5*M_PI;
      theta2 = M_PI - theta1;

      ROS_INFO_STREAM_NAMED("grasp","Theta1: " << theta1*RAD2DEG << " Theta2: " << theta2*RAD2DEG);

      // Calculate the theta1 for next time
      theta1 += M_PI / angle_resolution;

      grasp_pose.position.x = xb;
      grasp_pose.position.y = yb;
      grasp_pose.position.z = zb;

      Eigen::Quaternionf quat(Eigen::AngleAxis<float>(float(theta2), Eigen::Vector3f(0,1,0)));
      grasp_pose.orientation.x = quat.x();
      grasp_pose.orientation.y = quat.y();
      grasp_pose.orientation.z = quat.z();
      grasp_pose.orientation.w = quat.w();

      publishSphere(grasp_pose);
      publishArrow(grasp_pose);
      publishMesh(grasp_pose);


      ROS_INFO("sleeping\n");
      rate.sleep();


    }


  }

  // *********************************************************************************************************
  // Helper Function
  // *********************************************************************************************************
  void publishMesh(geometry_msgs::Pose &ee_pose)
  {
    ROS_INFO_STREAM("Mesh (" << ee_pose.position.x << ","<< ee_pose.position.y << ","<< ee_pose.position.z << ")");


    // -----------------------------------------------------------------------------------------------
    // Get end effector group

    // Create color
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1.0;
    marker_color.g = 0.1;
    marker_color.b = 0.1;
    marker_color.a = 0.5;

    ROS_INFO("getting robot state");

    // Get robot state
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

    ROS_INFO("getting link names");

    // Get link names that are in end effector
    const std::vector<std::string>
      &ee_link_names = robot_state.getJointStateGroup(EE_GROUP)->getJointModelGroup()->getLinkModelNames();
    ROS_INFO_STREAM_NAMED("grasp","Number of links in group " << EE_GROUP << ": " << ee_link_names.size());


    ROS_INFO("robot interaction");

    // Robot Interaction thing
    //    robot_interaction::RobotInteraction robot_interaction("some_name", robot_state, tf_);
    ///  RobotInteraction(const robot_model::RobotModelConstPtr &kmodel, const std::string &ns = "");
    robot_interaction::RobotInteraction robot_interaction( planning_scene_monitor_->getRobotModel() );

    ROS_INFO("decide active end effector");

    robot_interaction.decideActiveEndEffectors("arm");

    // EE Group?
    std::vector<robot_interaction::RobotInteraction::EndEffector> 
      active_eef = robot_interaction.getActiveEndEffectors();

    ROS_INFO_STREAM_NAMED("grasp","Number of end effectors: " << active_eef.size());
    
    if( !active_eef.size() )
    {
      ROS_ERROR_STREAM_NAMED("grasp","No end effectors found!");
      return;
    }
    robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

    // Create marker array
    visualization_msgs::MarkerArray marker_array;
    robot_state.getRobotMarkers(marker_array, ee_link_names, marker_color, eef.eef_group, ros::Duration());

    // Change pose?
    tf::Pose tf_root_to_link;
    try{
      tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link);
    }
    catch(...)
    {
      ROS_ERROR_STREAM_NAMED("grasp","Didn't find link state for " << eef.parent_link);
    }
    // Release the ptr count on the kinematic state
    //    robot_state.reset();

    // Create a generic marker for copying properties
    visualization_msgs::Marker generic_marker;
    generic_marker.header.frame_id = block_link;
    generic_marker.header.stamp = ros::Time::now();

    // Allow a transform from our pose to the end effector position
    geometry_msgs::Pose pose_to_eef;
    pose_to_eef.position.x = 0;
    pose_to_eef.position.y = 0;
    pose_to_eef.position.z = 0;
    pose_to_eef.orientation.x = 0;
    pose_to_eef.orientation.x = 0;
    pose_to_eef.orientation.x = 0;
    pose_to_eef.orientation.x = 1;

    ROS_INFO_STREAM_NAMED("grasp","Number of markers in end effector: " << marker_array.markers.size());

    // Process each link of the end effector
    for (std::size_t i = 0 ; i < marker_array.markers.size() ; ++i)
    {
      marker_array.markers[i].header = generic_marker.header;
      marker_array.markers[i].mesh_use_embedded_materials = true;

      // - - - - - - Do some math for the offset - - - - - -
      tf::Pose tf_root_to_im, tf_root_to_mesh, tf_pose_to_eef;
      tf::poseMsgToTF(ee_pose, tf_root_to_im);
      tf::poseMsgToTF(marker_array.markers[i].pose, tf_root_to_mesh);
      tf::poseMsgToTF(pose_to_eef, tf_pose_to_eef);
      tf::Pose tf_eef_to_mesh = tf_root_to_link.inverse() * tf_root_to_mesh;
      tf::Pose tf_im_to_mesh = tf_pose_to_eef * tf_eef_to_mesh;
      tf::Pose tf_root_to_mesh_new = tf_root_to_im * tf_im_to_mesh;
      tf::poseTFToMsg(tf_root_to_mesh_new, marker_array.markers[i].pose);
      // - - - - - - - - - - - - - - - - - - - - - - - - - -

      ROS_INFO_STREAM("Marker " << i << ":\n" << marker_array.markers[i]);


      marker_pub_.publish( marker_array.markers[i] );
      ros::Duration(2.0).sleep();
    }


    /*
    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Mesh";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://clam_description/stl/gripper_base_link.STL";

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.pose = pose;

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.r = 0.1;
    marker.color.g = 8.0;
    marker.color.b = 0.1;
    marker.color.a = 1.0;

    marker_pub_.publish( marker );
    */
  }

  void publishSphere(geometry_msgs::Pose &pose)
  {
    ROS_INFO_STREAM("Sphere (" << pose.position.x << ","<< pose.position.y << ","<< pose.position.z << ")");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = block_link;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Sphere";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    static int arrow_id = 0;
    marker.id = ++arrow_id;

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
    color.r = 1.0;
    color.g = 0.1;
    color.b = 0.1;
    color.a = 1.0;

    // Point
    geometry_msgs::Point point_a = pose.position;

    // Add the point pair to the line message
    marker.points.push_back( point_a );
    marker.colors.push_back( color );


    marker_pub_.publish( marker );
  }

  void publishArrow(geometry_msgs::Pose &pose)
  {
    ROS_INFO_STREAM("Arrow (" << pose.position.x << ","<< pose.position.y << ","<< pose.position.z << ")");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = block_link;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Arrow";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    static int arrow_id = 0;
    marker.id = ++arrow_id;

    marker.pose = pose;

    marker.scale.x = 0.2; // arrow width - but i would call this the length
    marker.scale.y = 0.01; // arrow height
    marker.scale.z = 0.01; // arrow length

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker_pub_.publish( marker );
  }


}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_generator_server");

  clam_block_manipulation::GraspGeneratorServer server("grap_gen");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::spin(); // keep the action server alive

  return 0;
}

