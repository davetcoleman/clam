/*
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Dave Coleman
 */

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>

// ClamArm
#include <clam_block_manipulation/PickPlaceAction.h>
#include <clam_controller/ClamArmAction.h>

// MoveIt
//#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/planning_models_loader/kinematic_model_loader.h>
#include <moveit/kinematic_state/conversions.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace clam_block_manipulation
{

static const std::string GROUP_NAME = "arm";
static const std::string ROBOT_DESCRIPTION="robot_description";

class PickPlaceServer
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // A ROS publisher
  ros::Publisher marker_pub_;

  // MoveGroup
  //  move_group_interface::MoveGroup group_;

  // Action Servers and Clients
  actionlib::SimpleActionServer<clam_block_manipulation::PickPlaceAction> as_;
  actionlib::SimpleActionClient<clam_controller::ClamArmAction> clam_arm_client_;

  // Action messages
  clam_controller::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server
  clam_block_manipulation::PickPlaceFeedback     feedback_;
  clam_block_manipulation::PickPlaceResult       result_;
  clam_block_manipulation::PickPlaceGoalConstPtr goal_;

  // Subscriber
  ros::Subscriber pick_place_sub_;

  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;

public:
  PickPlaceServer(const std::string name) :
    nh_("~"), as_(name, false),
    //    group_("arm"),
    clam_arm_client_("clam_arm", true)
  {

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO("[pick place] Waiting for the clam_arm action server");
    }

    // -----------------------------------------------------------------------------------------------
    // Rviz Visualizations
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Duration(0.5).sleep();

    // ---------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&PickPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickPlaceServer::preemptCB, this));
    as_.start();
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    ROS_INFO("[pick place] Received goal -----------------------------------------------");

    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    // Change the goal constraints on the servos to be less strict, so that the controllers don't die. this is a hack
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/elbow_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/shoulder_pan_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_arm_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45

    /*
    // Check if our listener has recieved a goal from the topic yet
    if (goal_->topic.length() < 1)
    {
    ROS_INFO("[pick place] Goal has already been recieved, start moving arm");
    pickAndPlace(goal_->pickup_pose, goal_->place_pose); // yes, start moving arm
    }
    else
    {
    ROS_INFO_STREAM("[pick place] Waiting for goal to be received on topic " << goal_->topic);
    pick_place_sub_ = nh_.subscribe(goal_->topic, 1, // no, wait for topic
    &PickPlaceServer::sendGoalFromTopic, this);
    }
    */


    // Skip perception
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose end_pose;

    start_pose.position.x = 0.2;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.2;

    end_pose.position.x = 0.2;
    end_pose.position.y = 0.1;
    end_pose.position.z = 0.2;

    pickAndPlace(start_pose, end_pose);
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick place] Got goal from topic! %s", goal_->topic.c_str());
    pickAndPlace(msg->poses[0], msg->poses[1]);
    pick_place_sub_.shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO("[pick place] Preempted");
    as_.setPreempted();
  }

  bool sendGoal(const geometry_msgs::Pose& pose)
  {
    // -----------------------------------------------------------------------------------------------
    // Planning Scene stuff
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();


    // -----------------------------------------------------------------------------------------------
    // Connect to move_group movegroup_actionion server
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> movegroup_action("move_group", false);
    ROS_INFO("[pick place] Connecting to move_group action server");
    movegroup_action.waitForServer();
    ROS_INFO("[pick place] Connected");



    // -----------------------------------------------------------------------------------------------
    // Move arm
    //    move_group_interface::MoveGroup group(GROUP_NAME);
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = GROUP_NAME;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = ros::Duration(5.0);


    // Position
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;
    ROS_INFO_STREAM("[pick place] Planning for x:" << x << " y:" << y << " z:" << z);

    // Orientation
    double qx = 0.00;
    double qy = 0.710502;
    double qz = -0.01755;
    double qw = 0.70346;

    bool gripperOpen = true;
    double x_offset = 0.15;


    // -------------------------------------------------------------------------------------------
    // Create goal state
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_link";
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.position.z = z;
    goal_pose.pose.orientation.x = qx;
    goal_pose.pose.orientation.y = qy;
    goal_pose.pose.orientation.z = qz;
    goal_pose.pose.orientation.w = qw;
    double tolerance_pose = 1e-3; // default: 1e-3... meters
    double tolerance_angle = 0.1; // default 1e-2... radians
    moveit_msgs::Constraints g0 =
      kinematic_constraints::constructGoalConstraints("gripper_roll_link", goal_pose,
                                                      tolerance_pose, tolerance_angle);


    g0.position_constraints[0].target_point_offset.x = x_offset;
    g0.position_constraints[0].target_point_offset.y = 0.0;
    g0.position_constraints[0].target_point_offset.z = 0.0;

    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = g0;

    // -------------------------------------------------------------------------------------------
    // Create start state
    kinematic_state::KinematicState start = scene.getCurrentState();


    //group.setStartState( start_state );
    //  group.setPositionTarget(0.22222, 0, 0.2);
    ROS_INFO_STREAM("[pick place] Planning for x:" << x << " y:" << y << " z:" << z);

    publishSphere(x, y, z);
    publishMesh(x, y, z + x_offset, qx, qy, qz, qw );


    // -------------------------------------------------------------------------------------------
    // Plan
    //move_group_interface::MoveGroup::Plan plan;
    movegroup_action.sendGoal(goal);
    sleep(5);

    if(!movegroup_action.waitForResult(ros::Duration(5.0)))
    {
      ROS_INFO_STREAM("Apparently returned early");
    }
    if (movegroup_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("It worked!");


    }
    else
    {
      ROS_WARN_STREAM("Fail: " << movegroup_action.getState().toString() << ": " << movegroup_action.getState().getText());
    }




    return true;
  }

  // Actually run the action
  void pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    geometry_msgs::Pose desired_pose = start_pose;

    // Wait for MoveArmAction to be ready ----------------------------------------------------------
    ROS_INFO("[pick place] PickAndPlace started");

    // -----------------------------------------------------------------------------------------------
    // Go to home position
    ROS_INFO("[pick place] Resetting arm to home position");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::RESET;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    clam_arm_client_.waitForResult(ros::Duration(20.0));
    //while(!clam_arm_client_.getState().isDone() && ros::ok())
    //  ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO("[pick place] Opening gripper");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Hover over block
    ROS_INFO("[pick place] Sending arm to pre-grasp position");
    desired_pose.position.z = 0.2;
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Lower over block
    ROS_INFO("[pick place] Sending arm to grasp position");
    desired_pose.position.z = desired_pose.position.z - 0.1;
    ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO("[pick place] Closing gripper");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::END_EFFECTOR_CLOSE;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Move Arm to new location
    ROS_INFO("[pick place] Sending arm to new position");
    desired_pose = end_pose;
    desired_pose.position.z = 0.2;
    ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendGoal(desired_pose))
      return;

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO("[pick place] Opening gripper");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Reset
    ROS_INFO("[pick place] Going to home position");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::RESET;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO("[pick place] Finished ------------------------------------------------");
    ROS_INFO(" ");
    as_.setSucceeded(result_);
  }

  // *********************************************************************************************************
  // Helper Function
  // *********************************************************************************************************
  void publishMesh(double x, double y, double z, double qx, double qy, double qz, double qw )
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
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

  void publishSphere(double x, double y, double z)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
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


    marker_pub_.publish( marker );
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_action_server");

  clam_block_manipulation::PickPlaceServer server("pick_place");
  ros::spin();

  return 0;
}

