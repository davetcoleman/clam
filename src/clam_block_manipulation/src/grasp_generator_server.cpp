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
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#define _USE_MATH_DEFINES

namespace clam_block_manipulation
{

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


  // *********************************************************************************************************
  // Helper Function
  // *********************************************************************************************************
  void publishMesh(double x, double y, double z, double qx, double qy, double qz, double qw )
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link;
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


    marker_pub_.publish( marker );
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
    double theta = 0;
    double angle_resolution = 20;

    geometry_msgs::Pose arrow_pose;

    ros::Rate rate(0.5);
    while (nh_.ok())
    {
      // Send Transform
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link, block_link));

      // Calculate grasp
      xb = r*cos(theta);
      zb = r*sin(theta);
      theta += M_PI / angle_resolution;

      arrow_pose.position.x = xb;
      arrow_pose.position.y = yb;
      arrow_pose.position.z = zb;

      Eigen::Quaternionf quat(Eigen::AngleAxis<float>(float(theta+M_PI), Eigen::Vector3f(0,-1,0)));
      arrow_pose.orientation.x = quat.x();
      arrow_pose.orientation.y = quat.y();
      arrow_pose.orientation.z = quat.z();
      arrow_pose.orientation.w = quat.w();

      publishSphere(xb, yb, zb);
      publishArrow(arrow_pose);

      ROS_INFO("sleeping\n");
      rate.sleep();
    }


  }

  void publishSphere(double x, double y, double z)
  {
    ROS_INFO_STREAM("Sphere (" << x << ","<< y << ","<< z << ")");

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
    color.r = 1.0;
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
    marker.id = 0;

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

