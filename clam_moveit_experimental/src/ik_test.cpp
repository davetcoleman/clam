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
   Desc:   Tests IK solver
*/

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Grasp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Clam
#include <clam_msgs/ClamArmAction.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit_msgs/PickupAction.h> // TODO: remove
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

// C++
#include <boost/thread.hpp>
#include <math.h>
#define _USE_MATH_DEFINES

namespace clam_block_manipulation
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_LINK = "gripper_roll_link";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_NAME = "end_effector";
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string MARKER_TOPIC = "/end_effector_marker";
static const std::string COLLISION_TOPIC = "/collision_object";
static const double RAD2DEG = 57.2957795;
static const double BLOCK_SIZE = 0.04;
// Class
class BlockGraspServer
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher rviz_marker_pub_;
  ros::Publisher collision_obj_pub_;

  // TODO: remove
  actionlib::SimpleActionClient<moveit_msgs::PickupAction> movegroup_action_;
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_;
  clam_msgs::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server

  // Action Servers and Clients
  /*  actionlib::SimpleActionServer<clam_msgs::PickPlaceAction> action_server_;

  // Action messages
  clam_msgs::PickPlaceFeedback     feedback_;
  clam_msgs::PickPlaceResult       result_;
  clam_msgs::PickPlaceGoalConstPtr goal_;
  */

  // Save collision object so we can delete it later
  moveit_msgs::CollisionObject chosen_block_object_;
  bool block_published_;

  // MoveIt Components
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Parameters from goal
  std::string base_link_;

  // TF Frame Transform stuff
  boost::thread tf_frame_thread_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform transform_;
  bool transform_is_set_;
  boost::mutex transform_mutex_;

  // Grasp axis orientation
  enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
  enum grasp_direction_t {UP, DOWN};

  // End Effector Markers
  bool ee_marker_is_loaded_; // determines if we have loaded the marker or not
  visualization_msgs::MarkerArray marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_;
  std::vector<geometry_msgs::Pose> marker_poses_;

public:

  // Constructor
  BlockGraspServer(const std::string name):
    nh_("~"),
    movegroup_action_("pickup", true), //TODO: remove
    clam_arm_client_("clam_arm", true), //TODO: remove
    ee_marker_is_loaded_(false),
    transform_is_set_(false),
    block_published_(false)
  {
    base_link_ = "base_link";

    // -----------------------------------------------------------------------------------------------
    // Rviz Visualizations
    rviz_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1);

    // -----------------------------------------------------------------------------------------------
    // Adding collision objects
    collision_obj_pub_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 1);

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group/Pickup action server
    while(!movegroup_action_.waitForServer(ros::Duration(4.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("ik_test","Waiting for the move_group/Pickup action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick place","Waiting for the clam_arm action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    tf_.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_, "dave_world"));

    // ---------------------------------------------------------------------------------------------
    // Check planning scene monitor
    if (planning_scene_monitor_->getPlanningScene())
    {
      /*
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
        planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      */
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "dave_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Planning scene not configured");
    }

    // ---------------------------------------------------------------------------------------------
    // Create a TF transform and start publishing in a seperate thread

    // Make the frame just be at the origin to start with
    //boost::thread tf_frame_thread_(boost::bind(&BlockGraspServer::tfFrameThread, this));

    // ---------------------------------------------------------------------------------------------
    // Test with randomly oriented block
    //srand (ros::Time::now());
    //double angle = rand() % M_PI;
    double angle = 0; //M_PI / 1.5;

    geometry_msgs::Pose block_pose;
    block_pose.position.x = 0.25;
    block_pose.position.y = 0.0;
    block_pose.position.z = 0.02;

    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    // ---------------------------------------------------------------------------------------------
    // Send home
    ROS_INFO_STREAM_NAMED("ik_test","Sending arm home");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Create the grasps
    pickAndPlace(block_pose);


    //    ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Register the goal and preempt callbacks
    /*    action_server_.registerGoalCallback(boost::bind(&BlockGraspServer::goalCB, this));
          action_server_.registerPreemptCallback(boost::bind(&BlockGraspServer::preemptCB, this));
          action_server_.start();
    */

    // Announce state
    //    ROS_INFO_STREAM_NAMED("pick_place", "Server ready.");
    //    ROS_INFO_STREAM_NAMED("pick_place", "Waiting for pick command...");
  }

  // Destructor
  ~BlockGraspServer()
  {
    // join the thread back before exiting
    //tf_frame_thread_.join();
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

  // Run pick and place
  bool pickAndPlace( geometry_msgs::Pose& block_pose )
  {
    // ---------------------------------------------------------------------------------------------
    // Create the transform from block_frame to base_frame
    tf::Pose tf_block_pose;
    tf::poseMsgToTF(block_pose, tf_block_pose);
    transform_ = tf_block_pose;

    // ---------------------------------------------------------------------------------------------
    // Add the block to the planning scene
    //    createCollisionObject(block_pose, chosen_block_object_);

    // ---------------------------------------------------------------------------------------------
    // Generate grasps
    std::vector<moveit_msgs::Grasp> possible_grasps;     // List of possible block grasps
    generateGrasps( block_pose, possible_grasps );

    // Visualize results
    visualizeGrasps(possible_grasps, block_pose);

    // Filter grasp poses
    if( !filterGrasps( possible_grasps ) )
      return false;

    ROS_INFO_STREAM_NAMED("ik_test","Possible grasps filtered to " << possible_grasps.size() << " options.");

    // Visualize results
    visualizeGrasps(possible_grasps, block_pose);


    // Plan the results
    //executeGrasps(possible_grasps, block_pose);

    return true;


    /*
      std::vector<moveit_msgs::Grasp> single_grasp;

      // Loop through and plan for each individual grasp
      for( int i = 3; i < possible_grasps.size(); ++i)
      {
      ROS_INFO_STREAM_NAMED("ik_test","Trying grasp " << i );

      // ---------------------------------------------------------------------------------------------
      // Choose one
      single_grasp.clear();
      single_grasp.push_back(possible_grasps[i]);

      // Visualize results
      visualizeGrasps(single_grasp, block_pose);

      // Plan the results
      executeGrasps(single_grasp, block_pose);

      // ---------------------------------------------------------------------------------------------
      // Send home
      ROS_INFO_STREAM_NAMED("ik_test","Sending arm home");
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

      // Remove the attached object
      //deleteCollisionObject(chosen_block_object_);
      }
    */

    return true;
  }

  // Create all possible grasp positions for a block
  void generateGrasps(geometry_msgs::Pose& block_pose, std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    // ---------------------------------------------------------------------------------------------
    // Calculate grasps in two axis

    //generateAxisGrasps( possible_grasps, X_AXIS, DOWN );
    //generateAxisGrasps( possible_grasps, X_AXIS, UP );
    //generateAxisGrasps( possible_grasps, Y_AXIS, DOWN );
    //generateAxisGrasps( possible_grasps, Y_AXIS, UP );

    generateSimpleGrasp(possible_grasps);
  }


  // Create grasp that lowers a little from zero position
  bool generateSimpleGrasp(std::vector<moveit_msgs::Grasp>& possible_grasps)

  {
    // Create re-usable blank pose
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.stamp = ros::Time::now();
    grasp_pose.header.frame_id = base_link_;


    // Get the robot state
    const robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    const Eigen::Affine3d ee_transform = robot_state.getLinkState(EE_LINK)->getGlobalLinkTransform();

    //Eigen::Matrix4d m = ee_transform.matrix();
    //ROS_INFO_STREAM("Link state: \n" << m);

    // Convert affine transform to tf pose to geometry_msgs pose
    tf::Pose ee_tf_pose;
    tf::poseEigenToTF(ee_transform, ee_tf_pose);
    tf::poseTFToMsg(ee_tf_pose, grasp_pose.pose);

    // ---------------------------------------------------------------------------------------------
    // Generate a set of grasp that are slowly lower
    //    while( grasp_pose.pose.position.z > 0.1 )
    //    {
    // Lower the pose
    grasp_pose.pose.position.z -= 0.1;

    // ---------------------------------------------------------------------------------------------
    // Create a Grasp message
    moveit_msgs::Grasp new_grasp;
    new_grasp.grasp_pose = grasp_pose;

    // ---------------------------------------------------------------------------------------------
    // Add to possible grasps
    possible_grasps.push_back(new_grasp);
    //    }

  }


  // Create grasp positions in one axis
  bool generateAxisGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
                          grasp_direction_t direction )
  {

    // ---------------------------------------------------------------------------------------------
    // moveit_msgs:Grasp parameters

    // Create re-usable approach motion
    moveit_msgs::GripperTranslation gripper_approach;
    gripper_approach.direction.header.stamp = ros::Time::now();
    gripper_approach.direction.header.frame_id = base_link_;
    gripper_approach.direction.vector.x = 1;
    gripper_approach.direction.vector.y = 0;
    gripper_approach.direction.vector.z = 0; // Approach direction (negative z axis)
    //gripper_approach.desired_distance = .050; // The distance the origin of a robot link needs to travel
    //gripper_approach.min_distance = .025; // half of the desired? Untested.
    gripper_approach.desired_distance = .050; // The distance the origin of a robot link needs to travel
    gripper_approach.min_distance = .025; // half of the desired? Untested.

    // Create re-usable retreat motion
    moveit_msgs::GripperTranslation gripper_retreat;
    gripper_retreat.direction.header.stamp = ros::Time::now();
    gripper_retreat.direction.header.frame_id = base_link_;
    gripper_retreat.direction.vector.x = 0;
    gripper_retreat.direction.vector.y = 0;
    gripper_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
    gripper_retreat.desired_distance = .050; // The distance the origin of a robot link needs to travel
    gripper_retreat.min_distance = .025; // half of the desired? Untested.

    // Create re-usable blank pose
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.stamp = ros::Time::now();
    grasp_pose.header.frame_id = base_link_;

    // ---------------------------------------------------------------------------------------------
    // Variables needed for calculations
    double radius = 0.15;
    double xb;
    double yb = 0.0; // stay in the y plane of the block
    double zb;
    double angle_resolution = 8.0;
    double theta1 = 0.0; // Where the point is located around the block
    double theta2 = 0.0; // UP 'direction'

    // Gripper direction (UP/DOWN) rotation. UP set by default
    if( direction == DOWN )
    {
      theta2 = M_PI;
    }

    /* Developer Note:
     * Create angles 180 degrees around the chosen axis at given resolution
     * We create the grasps in the reference frame of the block, then convert it to the base link
     *
     */
    for(int i = 0; i <= angle_resolution; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("ik_test", "Generating grasp " << i );

      // Calculate grasp
      xb = radius*cos(theta1);
      zb = radius*sin(theta1);

      //ROS_DEBUG_STREAM_NAMED("ik_test","Theta1: " << theta1*RAD2DEG);

      Eigen::Matrix3d rotation_matrix;
      switch(axis)
      {
      case X_AXIS:
        grasp_pose.pose.position.x = yb;
        grasp_pose.pose.position.y = xb;
        grasp_pose.pose.position.z = zb;

        rotation_matrix = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        break;
      case Y_AXIS:
        grasp_pose.pose.position.x = xb;
        grasp_pose.pose.position.y = yb;
        grasp_pose.pose.position.z = zb;

        rotation_matrix =
          Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        break;
      case Z_AXIS:
        ROS_ERROR_STREAM_NAMED("ik_test","Z Axis not implemented!");
        return false;

        break;
      }

      Eigen::Quaterniond quat(rotation_matrix);
      grasp_pose.pose.orientation.x = quat.x();
      grasp_pose.pose.orientation.y = quat.y();
      grasp_pose.pose.orientation.z = quat.z();
      grasp_pose.pose.orientation.w = quat.w();

      // Calculate the theta1 for next time
      theta1 += M_PI / angle_resolution;

      // ---------------------------------------------------------------------------------------------
      // Create a Grasp message
      moveit_msgs::Grasp new_grasp;

      // A name for this grasp
      static int grasp_id = 0;
      new_grasp.id = "ik_test" + boost::lexical_cast<std::string>(grasp_id);

      // The internal posture of the hand for the pre-grasp
      // only positions are used
      //sensor_msgs/JointState pre_grasp_posture
      new_grasp.pre_grasp_posture.header.frame_id = base_link_;

      // The internal posture of the hand for the grasp
      // positions and efforts are used
      //sensor_msgs/JointState grasp_posture
      new_grasp.grasp_posture.header.frame_id = base_link_;

      // Convert pose to TF, transform it, then convert it back to geometry_msg
      tf::Pose tf_grasp_pose_block_frame;
      tf::Pose tf_grasp_pose_base_frame;
      tf::poseMsgToTF(grasp_pose.pose, tf_grasp_pose_block_frame);
      tf_grasp_pose_base_frame = transform_ * tf_grasp_pose_block_frame;
      tf::poseTFToMsg(tf_grasp_pose_base_frame, grasp_pose.pose);

      // The position of the end-effector for the grasp relative to a reference frame
      // (that is always specified elsewhere, not in this message)
      new_grasp.grasp_pose = grasp_pose;

      // The estimated probability of success for this grasp, or some other
      // measure of how "good" it is.
      new_grasp.grasp_quality = 1;

      // The approach motion
      new_grasp.approach = gripper_approach;

      // The retreat motion
      new_grasp.retreat = gripper_retreat;

      // the maximum contact force to use while grasping (<=0 to disable)
      new_grasp.max_contact_force = 0;

      // an optional list of obstacles that we have semantic information about
      // and that can be touched/pushed/moved in the course of grasping
      //string[] allowed_touch_objects

      // Add to vector
      possible_grasps.push_back(new_grasp);
    }


    ROS_INFO_STREAM_NAMED("ik_test", "Generated " << possible_grasps.size() << " grasps sucessfully." );
    return true;
  }

  void filterNthGrasp(std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    // Only choose the 4th grasp
    std::vector<moveit_msgs::Grasp> single_grasp;
    single_grasp.push_back(possible_grasps[4]);
    possible_grasps = single_grasp;

    ROS_INFO_STREAM_NAMED("ik_test","Possible grasps filtered to " << possible_grasps.size() << " options.");
  }

  // Choose the 1st grasp that is kinematically feasible
  bool filterGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    // -----------------------------------------------------------------------------------------------
    // Error check
    if( possible_grasps.empty() )
    {
      ROS_ERROR_NAMED("ik_test","Unable to filter grasps because vector is empty");
      return false;
    }

    // -----------------------------------------------------------------------------------------------
    // Get the IK solver
    boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kinematics_plugin_loader;
    kinematics_plugin_loader.reset(new kinematics_plugin_loader::KinematicsPluginLoader());
    kinematics_plugin_loader::KinematicsLoaderFn
      kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();

    const robot_model::JointModelGroup* planning_group
      = planning_scene_monitor_->getPlanningScene()->getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME);

    kinematics::KinematicsBasePtr kin_solver = kinematics_allocator(planning_group);

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find first one that has solution

    std::vector<moveit_msgs::Grasp> feasible_grasp;

    int num_ik_solutions = 0;
    for (std::vector<moveit_msgs::Grasp>::iterator grasp_it = possible_grasps.begin();
         grasp_it!=possible_grasps.end(); ++grasp_it)
    {

      // -----------------------------------------------------------------------------------------------
      // Declare needed IK solver vars
      geometry_msgs::Pose ik_pose = (*grasp_it).grasp_pose.pose; //possible_grasps[0].grasp_pose.pose;
      std::vector<double> ik_seed_state;
      ik_seed_state.push_back(0);
      ik_seed_state.push_back(0);
      ik_seed_state.push_back(0);
      ik_seed_state.push_back(0);
      ik_seed_state.push_back(0);
      ik_seed_state.push_back(0);
      //      ik_seed_state.push_back(0);
      std::vector<double> solution;
      moveit_msgs::MoveItErrorCodes error_code;

      // Test it with IK
      //      kin_solver->getPositionIK(ik_pose, ik_seed_state, solution, error_code);
      kin_solver->searchPositionIK(ik_pose, ik_seed_state, 100, solution, error_code);

      // Results
      if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
      {
        ROS_INFO_STREAM_NAMED("ik_test","Final IK Solution:");
        int i = 0;
        for (std::vector<double>::iterator solution_it = solution.begin();
             solution_it!=solution.end(); ++solution_it)
        {
          std::cout << "Joint " << i << ": " << *solution_it << std::endl;
          ++i;
        }

        feasible_grasp.push_back(*grasp_it);

        // Show this kinematic solution in rviz
        //        robot_state::JointStateGroup* joint_state_group =
        planning_scene_monitor_->getPlanningScene()->getCurrentState().getJointStateGroup(PLANNING_GROUP_NAME)
          ->setVariableValues(solution);
        planning_scene_monitor_->updateFrameTransforms();
        planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

        ROS_INFO_STREAM_NAMED("ik_test","Set joint group values, updating frame transform and sleeping...");
        ros::Duration(2.0).sleep();

        num_ik_solutions ++;
      }
      else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      {
        ROS_WARN_STREAM_NAMED("ik_test","Unable to find IK solution for pose.");
      }
      else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
      {
        ROS_WARN_STREAM_NAMED("ik_test","Unable to find IK solution for pose: Timed Out.");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ik_test","IK solution error: MoveItErrorCodes.msg = " << error_code);
      }
    }

    //    ROS_ERROR_STREAM_NAMED("ik_test","Unable to find any IK solution.");

    ROS_INFO_STREAM_NAMED("ik_test", "Found " << num_ik_solutions << " ik solutions out of " <<
                          possible_grasps.size() );

    possible_grasps = feasible_grasp;

    return true;
  }

  void createCollisionObject(const geometry_msgs::Pose& block_pose, moveit_msgs::CollisionObject& block_object)
  {
    if( block_published_ )
    {
      return; // only publish the block once!
    }

    ROS_INFO_STREAM_NAMED("ik_test","Creating the collision object");
    // ---------------------------------------------------------------------------------------------
    // Create Solid Primitive
    shape_msgs::SolidPrimitive block_shape;

    // type of the shape
    block_shape.type = shape_msgs::SolidPrimitive::BOX;

    // dimensions of the shape
    //    block_shape.dimensions.resize(3);
    block_shape.dimensions.push_back(BLOCK_SIZE); // x
    block_shape.dimensions.push_back(BLOCK_SIZE); // y
    block_shape.dimensions.push_back(BLOCK_SIZE); // z

    // ---------------------------------------------------------------------------------------------
    // Add the block to the collision environment

    // a header, used for interpreting the poses
    block_object.header.frame_id = base_link_;
    block_object.header.stamp = ros::Time::now();

    // the id of the object
    static int block_id = 0;
    block_object.id = "Block" + boost::lexical_cast<std::string>(block_id);

    // the the collision geometries associated with the object;
    // their poses are with respect to the specified header

    // solid geometric primitives
    //shape_msgs/SolidPrimitive[] primitives // TODO?
    block_object.primitives.push_back(block_shape);

    //geometry_msgs/Pose[] primitive_poses
    block_object.primitive_poses.push_back( block_pose );

    // meshes
    //shape_msgs/Mesh[] meshes
    //geometry_msgs/Pose[] mesh_poses

    // bounding planes (equation is specified, but the plane can be oriented using an additional pose)
    //shape_msgs/Plane[] planes
    //geometry_msgs/Pose[] plane_poses

    // Operation to be performed
    block_object.operation = moveit_msgs::CollisionObject::ADD; // Puts the object into the environment or updates the object if already added

    // Send the object
    collision_obj_pub_.publish(block_object);
    block_published_ = true;
    ROS_INFO_STREAM_NAMED("ik_test","Collision object published for addition");
  }

  // *Requires that the object already be created
  void deleteCollisionObject(moveit_msgs::CollisionObject& block_object)
  {
    // Operation to be performed
    block_object.operation = moveit_msgs::CollisionObject::REMOVE;

    // Send the object
    block_published_ = false;
    collision_obj_pub_.publish(block_object);
    ROS_INFO_STREAM_NAMED("ik_test","Collision object published for removal");
  }

  void executeGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                     const geometry_msgs::Pose& block_pose)
  {
    ROS_INFO_STREAM_NAMED("ik_test","Creating Pickup Goal");


    //ROS_INFO_STREAM_NAMED("ik_test","Temp done");
    //return;

    // ---------------------------------------------------------------------------------------------
    // Create PlanningOptions
    moveit_msgs::PlanningOptions options;

    // The diff to consider for the planning scene (optional)
    //PlanningScene planning_scene_diff

    // If this flag is set to true, the action
    // returns an executable plan in the response but does not attempt execution
    options.plan_only = false;

    // If this flag is set to true, the action of planning &
    // executing is allowed to look around  (move sensors) if
    // it seems that not enough information is available about
    // the environment
    options.look_around = false;

    // If this value is positive, the action of planning & executing
    // is allowed to look around for a maximum number of attempts;
    // If the value is left as 0, the default value is used, as set
    // with dynamic_reconfigure
    //int32 look_around_attempts

    // If set and if look_around is true, this value is used as
    // the maximum cost allowed for a path to be considered executable.
    // If the cost of a path is higher than this value, more sensing or
    // a new plan needed. If left as 0.0 but look_around is true, then
    // the default value set via dynamic_reconfigure is used
    //float64 max_safe_execution_cost

    // If the plan becomes invalidated during execution, it is possible to have
    // that plan recomputed and execution restarted. This flag enables this
    // functionality
    options.replan = false;

    // The maximum number of replanning attempts
    //int32 replan_attempts

    // ---------------------------------------------------------------------------------------------
    // Create and populate the goal
    moveit_msgs::PickupGoal goal;

    // An action for picking up an object

    // The name of the object to pick up (as known in the planning scene)
    goal.target_name = chosen_block_object_.id;

    // which group should be used to plan for pickup
    goal.group_name = PLANNING_GROUP_NAME;

    // which end-effector to be used for pickup (ideally descending from the group above)
    goal.end_effector = EE_NAME;

    // a list of possible grasps to be used. At least one grasp must be filled in
    goal.possible_grasps.resize(possible_grasps.size());
    goal.possible_grasps = possible_grasps;

    // the name that the support surface (e.g. table) has in the collision map
    // can be left empty if no name is available
    //string collision_support_surface_name

    // whether collisions between the gripper and the support surface should be acceptable
    // during move from pre-grasp to grasp and during lift. Collisions when moving to the
    // pre-grasp location are still not allowed even if this is set to true.
    goal.allow_gripper_support_collision = true;

    // The names of the links the object to be attached is allowed to touch;
    // If this is left empty, it defaults to the links in the used end-effector
    //string[] attached_object_touch_links

    // Optional constraints to be imposed on every point in the motion plan
    //Constraints path_constraints

    // an optional list of obstacles that we have semantic information about
    // and that can be touched/pushed/moved in the course of grasping;
    // CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.
    //string[] allowed_touch_objects

    // The maximum amount of time the motion planner is allowed to plan for
    goal.allowed_planning_time = 10.0; // seconds?

    // Planning options
    goal.planning_options = options;

    //ROS_INFO_STREAM_NAMED("ik_test","Pause");
    //ros::Duration(5.0).sleep();

    // ---------------------------------------------------------------------------------------------
    // Send the grasp to move_group/Pickup
    ROS_INFO_STREAM_NAMED("ik_test","Sending pick action to move_group/Pickup");

    movegroup_action_.sendGoal(goal);
    ros::Duration(5.0).sleep();

    if(!movegroup_action_.waitForResult(ros::Duration(20.0)))
    {
      ROS_INFO_STREAM_NAMED("ik_test","Returned early?");
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("ik_test","Plan successful!");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("ik_test","FAILED: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
    }
  }

  // Show all grasps in Rviz
  void visualizeGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                       const geometry_msgs::Pose& block_pose)
  {
    ROS_INFO_STREAM_NAMED("ik_test","Visualizing all generating grasp poses on topic " << MARKER_TOPIC);
    ros::Rate rate(1.0);

    for(std::vector<moveit_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
        grasp_it < possible_grasps.end(); ++grasp_it)
    {
      ROS_DEBUG_STREAM_NAMED("ik_test","Visualizing grasp pose");
      geometry_msgs::Pose grasp_pose = grasp_it->grasp_pose.pose;
      publishSphere(grasp_pose);
      publishArrow(grasp_pose);
      //publishEEMarkers(grasp_pose);
      //publishBlock(block_pose, BLOCK_SIZE - 0.001);
      //rate.sleep();
    }
  }

  // *********************************************************************************************************
  // Helper Function
  // *********************************************************************************************************

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
    robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(EE_GROUP);
    if( joint_state_group == NULL ) // make sure EE_GROUP exists
    {
      ROS_ERROR_STREAM_NAMED("ik_test","Unable to find joint state group " << EE_GROUP );
      return false;
    }


    // Get link names that are in end effector
    const std::vector<std::string>
      &ee_link_names = joint_state_group->getJointModelGroup()->getLinkModelNames();
    ROS_DEBUG_STREAM_NAMED("ik_test","Number of links in group " << EE_GROUP << ": " << ee_link_names.size());

    // Robot Interaction - finds the end effector associated with a planning group
    robot_interaction::RobotInteraction robot_interaction( planning_scene_monitor_->getRobotModel() );

    // Decide active end effectors
    robot_interaction.decideActiveEndEffectors(PLANNING_GROUP_NAME);

    // Get active EE
    std::vector<robot_interaction::RobotInteraction::EndEffector>
      active_eef = robot_interaction.getActiveEndEffectors();
    ROS_DEBUG_STREAM_NAMED("ik_test","Number of active end effectors: " << active_eef.size());
    if( !active_eef.size() )
    {
      ROS_ERROR_STREAM_NAMED("ik_test","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
      return false;
    }

    // Just choose the first end effector TODO: better logic?
    robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

    // -----------------------------------------------------------------------------------------------
    // Get EE link markers for Rviz
    robot_state.getRobotMarkers(marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
    ROS_DEBUG_STREAM_NAMED("ik_test","Number of rviz markers in end effector: " << marker_array_.markers.size());

    // Change pose from Eigen to TF
    try{
      tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
    }
    catch(...)
    {
      ROS_ERROR_STREAM_NAMED("ik_test","Didn't find link state for " << eef.parent_link);
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

  void publishEEMarkers(geometry_msgs::Pose &grasp_pose)
  {
    //ROS_INFO_STREAM("Mesh (" << grasp_pose.position.x << ","<< grasp_pose.position.y << ","<< grasp_pose.position.z << ")");

    // -----------------------------------------------------------------------------------------------
    // Make sure EE Marker is loaded
    if( !ee_marker_is_loaded_ )
    {
      ROS_INFO_STREAM_NAMED("ik_test","Loading end effector rviz markers");
      if( !loadEEMarker() )
      {
        ROS_WARN_STREAM_NAMED("ik_test","Unable to publish EE marker");
        return;
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
      //marker_array_.markers[i].lifetime = ros::Duration(30.0);

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

  }

  void publishSphere(geometry_msgs::Pose &pose)
  {
    //ROS_INFO_STREAM("Sphere (" << pose.position.x << ","<< pose.position.y << ","<< pose.position.z << ")");

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

    static int id = 0;
    marker.id = ++id;

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

    marker.lifetime = ros::Duration(30.0);

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


    rviz_marker_pub_.publish( marker );
    ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz
  }

  void publishArrow(geometry_msgs::Pose &pose)
  {
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

    marker.lifetime = ros::Duration(30.0);

    rviz_marker_pub_.publish( marker );
    ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz
  }

  void publishBlock(const geometry_msgs::Pose &pose, const double& block_size)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Block";
    marker.id = 1;

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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    rviz_marker_pub_.publish( marker );
    ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_grasp_server");

  clam_block_manipulation::BlockGraspServer server("grap_gen");

  // Allow the action server to recieve and send ros messages
  //  ros::spin(); // keep the action server alive
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

