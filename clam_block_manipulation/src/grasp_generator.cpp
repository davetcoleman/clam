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

#include "grasp_generator.h"

namespace clam_block_manipulation
{

// Constructor
GraspGenerator::GraspGenerator( planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                std::string base_link ):
  planning_scene_monitor_(planning_scene_monitor),
  base_link_(base_link),
  nh_("~"),
  ee_marker_is_loaded_(false),
  marker_lifetime_(ros::Duration(60.0))
{
  ik_cache_.reset(new simple_cache::SimpleCache(7, false)); // TODO: dynamically set 7 (num of joints) based on robot

  base_link_ = "base_link";

  // -----------------------------------------------------------------------------------------------
  // Rviz Visualizations
  rviz_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1);

  // -----------------------------------------------------------------------------------------------
  // Adding collision objects
  collision_obj_pub_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 1);

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
    ROS_ERROR_STREAM_NAMED("grasp","Planning scene not configured");
  }


}

// Create all possible grasp positions for a block
bool GraspGenerator::generateGrasps(const geometry_msgs::Pose& block_pose, std::vector<manipulation_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the block's frame (center of block) to /base_link
  tf::Pose tf_block_pose;
  tf::poseMsgToTF(block_pose, tf_block_pose);
  transform_ = tf_block_pose;

  // ---------------------------------------------------------------------------------------------
  // Show block
  publishBlock(block_pose, BLOCK_SIZE);

  // ---------------------------------------------------------------------------------------------
  // Generate grasps

  // Calculate grasps in two axis in both directions
  generateAxisGrasps( possible_grasps, X_AXIS, DOWN );
  generateAxisGrasps( possible_grasps, X_AXIS, UP );
  generateAxisGrasps( possible_grasps, Y_AXIS, DOWN );
  generateAxisGrasps( possible_grasps, Y_AXIS, UP );

  // Visualize results
  //visualizeGrasps(possible_grasps, block_pose);

  filterFromCache( possible_grasps );

  // Filter grasp poses
  if( !filterGrasps( possible_grasps ) )
    return false;

  //ROS_INFO_STREAM_NAMED("grasp","Possible grasps filtered to " << possible_grasps.size() << " options.");

  // Visualize results
  visualizeGrasps(possible_grasps, block_pose);

  return true;
}

bool GraspGenerator::chooseBestGrasp( const std::vector<manipulation_msgs::Grasp>& possible_grasps, manipulation_msgs::Grasp& chosen )
{
  // TODO: better logic here
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("grasp","There are no grasps to choose from");
    return false;
  }
  chosen = possible_grasps[0]; // just choose first one
}

// Create grasp positions in one axis
bool GraspGenerator::generateAxisGrasps(std::vector<manipulation_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
                                        grasp_direction_t direction )
{

  // ---------------------------------------------------------------------------------------------
  // manipulation_msgs:Grasp parameters

  // Create re-usable approach motion
  manipulation_msgs::GripperTranslation gripper_approach;
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
  manipulation_msgs::GripperTranslation gripper_retreat;
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
    //ROS_DEBUG_STREAM_NAMED("grasp", "Generating grasp " << i );

    // Calculate grasp
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    //ROS_DEBUG_STREAM_NAMED("grasp","Theta1: " << theta1*RAD2DEG);

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
      ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
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
    manipulation_msgs::Grasp new_grasp;

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);

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


  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps sucessfully." );
  return true;
}

bool GraspGenerator::filterNthGrasp(std::vector<manipulation_msgs::Grasp>& possible_grasps, int n)
{
  // Only choose the 4th grasp
  std::vector<manipulation_msgs::Grasp> single_grasp;
  single_grasp.push_back(possible_grasps[n]);
  possible_grasps = single_grasp;

  ROS_INFO_STREAM_NAMED("grasp","Possible grasps filtered to " << possible_grasps.size() << " options.");
  return true;
}

// Grasp cache lookup
bool GraspGenerator::filterFromCache(std::vector<manipulation_msgs::Grasp>& possible_grasps)
{
  // TODO remove this

  return false;
}

// Return grasps that are kinematically feasible
bool GraspGenerator::filterGrasps(std::vector<manipulation_msgs::Grasp>& possible_grasps)
{
  // -----------------------------------------------------------------------------------------------
  // Error check
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("grasp","Unable to filter grasps because vector is empty");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Get the IK solver
  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kinematics_plugin_loader;
  kinematics_plugin_loader.reset(new kinematics_plugin_loader::KinematicsPluginLoader());
  kinematics_plugin_loader::KinematicsLoaderFn
    kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();

  const robot_model::JointModelGroup* joint_model_group
    = planning_scene_monitor_->getPlanningScene()->getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME);

  const robot_state::JointStateGroup* joint_state_group
    = planning_scene_monitor_->getPlanningScene()->getCurrentState().getJointStateGroup(PLANNING_GROUP_NAME);

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();
  {

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find those that are kinematically feasible
    std::vector<manipulation_msgs::Grasp> filtered_grasps;

    boost::thread_group bgroup; // create a group of threads
    boost::mutex lock; // used for sharing the same data structures

    // how many cores does this computer have and how many do we need?
    int num_threads = boost::thread::hardware_concurrency();
    if( num_threads > possible_grasps.size() )
      num_threads = possible_grasps.size();
    //num_threads = 1;

    ROS_INFO_STREAM_NAMED("grasp", "IK checking on " << num_threads << " threads...");

    // split up the work between threads
    double num_grasps_per_thread = double(possible_grasps.size()) / num_threads;
    //ROS_INFO_STREAM("total grasps " << possible_grasps.size() << " per thead: " << num_grasps_per_thread);

    int grasps_id_start;
    int grasps_id_end = 0;

    for(int i = 0; i < num_threads; ++i)
    {
      grasps_id_start = grasps_id_end;
      grasps_id_end = ceil(num_grasps_per_thread*(i+1));
      if( grasps_id_end >= possible_grasps.size() )
        grasps_id_end = possible_grasps.size();
      //ROS_INFO_STREAM_NAMED("grasp","low " << grasps_id_start << " high " << grasps_id_end);

      IkThreadStruct tc(possible_grasps, filtered_grasps, grasps_id_start, grasps_id_end, joint_model_group,
                        joint_state_group, kinematics_allocator, ik_cache_, &lock, i);
      bgroup.create_thread( boost::bind( &GraspGenerator::filterGraspThread, this, tc ) );
    }

    bgroup.join_all(); // wait for all threads to finish

    ROS_INFO_STREAM_NAMED("grasp", "Found " << filtered_grasps.size() << " ik solutions out of " <<
                          possible_grasps.size() );
    ROS_INFO_STREAM_NAMED("grasp","ik cache has " << ik_cache_->getSize() << " entries");

    possible_grasps = filtered_grasps;

  }
  // End Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("grasp","RESULTS:");
  std::cout << duration << "\t" << possible_grasps.size() << "\n";

  return false;
}

// Thread for checking part of the possible grasps list
void GraspGenerator::filterGraspThread(IkThreadStruct ik_thread_struct)
{
  // Create this thread's own ik solver instance
  kinematics::KinematicsBasePtr kin_solver =
    ik_thread_struct.kinematics_allocator_(ik_thread_struct.joint_model_group_);

  // Seed state - start at zero - TODO: not zero
  std::vector<double> ik_seed_state(7); // fill with zeros
  /*
    std::vector<double> ik_seed_state;
    ik_seed_state.push_back(-3.46603e-06);
    ik_seed_state.push_back(-0.312857);
    ik_seed_state.push_back(1.75185);
    ik_seed_state.push_back(-2.0385e-06);
    ik_seed_state.push_back(1.6476);
    ik_seed_state.push_back(-0.523598);
  */

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::Pose* ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    ROS_DEBUG_STREAM_NAMED("grasp", "Checking grasp #" << i);

    // Pointer to current pose
    ik_pose = &ik_thread_struct.possible_grasps_[i].grasp_pose.pose;

    // Get seed state from cache if one is available
    simple_cache::results_t cache_result = ik_thread_struct.ik_cache_->get(*ik_pose, ik_seed_state);
    if( cache_result == simple_cache::SUCCESS )
    {
      ROS_INFO_STREAM_NAMED("grasp","ik result found from cache!!");
    }

    // Test it with IK
    kin_solver->getPositionIK(*ik_pose, ik_seed_state, solution, error_code);

    /*
    // Do attempts # of ik calls
    unsigned int attempts = 5;
    double timeout = 5.0; //ik_thread_struct.joint_state_group_->getDefaultIKTimeout();

    bool first_seed = true;
    for (unsigned int attempt = 0 ; attempt < attempts ; ++attempt)
    {
    // the first seed is the initial state
    if (first_seed)
    {
    // use original ik_seed_state
    first_seed = false;
    }
    else
    {
    // sample a random seed
    random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
    std::vector<double> random_values;
    ik_thread_struct.joint_model_group_->getVariableRandomValues(rng, random_values);
    //ik_seed_state = random_values;
    }

    //ROS_DEBUG_STREAM("seed state:");
    //std::copy(ik_seed_state.begin(),ik_seed_state.end(), std::ostream_iterator<double>(std::cout, "\n"));

    // compute the IK solution
    if (kin_solver->searchPositionIK(*ik_pose, ik_seed_state, timeout, solution, error_code) )
    {
    // we found a solution, so we can actually quit!
    ROS_INFO_STREAM("Found solution on " << attempt << " attempt");
    break;
    }
    else
    {
    ROS_INFO_STREAM("No solution found solution on " << attempt << " attempt");
    }
    }
    */

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      ROS_INFO_STREAM_NAMED("grasp","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      //ik_seed_state = solution;

      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );

        // if the cache did not have an entry, add it
        if( cache_result != simple_cache::SUCCESS )
        {
          ROS_INFO_STREAM_NAMED("grasp","inserting into ik cache");
          ik_thread_struct.ik_cache_->insert(*ik_pose, solution);
        }
      }

      // TODO: is this thread safe? (prob not)
      publishSphere(*ik_pose);
      publishArrow(*ik_pose);
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
      ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose: Timed Out.");
    else
      ROS_INFO_STREAM_NAMED("grasp","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }
}

// Show all grasps in Rviz
void GraspGenerator::visualizeGrasps(const std::vector<manipulation_msgs::Grasp>& possible_grasps,
                                     const geometry_msgs::Pose& block_pose)
{
  ROS_INFO_STREAM_NAMED("grasp","Visualizing all generating grasp poses on topic " << MARKER_TOPIC);
  ros::Rate rate(1.0);

  for(std::vector<manipulation_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
      grasp_it < possible_grasps.end(); ++grasp_it)
  {
    geometry_msgs::Pose grasp_pose = grasp_it->grasp_pose.pose;

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose\n" << grasp_pose);
    ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose");

    publishSphere(grasp_pose);
    publishArrow(grasp_pose);
    //publishEEMarkers(grasp_pose);
    publishBlock(block_pose, BLOCK_SIZE);
    //rate.sleep();
  }
}

// *********************************************************************************************************
// Helper Function
// *********************************************************************************************************

// Call this once at begining to load the robot marker
bool GraspGenerator::loadEEMarker()
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
    ROS_ERROR_STREAM_NAMED("grasp","Unable to find joint state group " << EE_GROUP );
    return false;
  }


  // Get link names that are in end effector
  const std::vector<std::string>
    &ee_link_names = joint_state_group->getJointModelGroup()->getLinkModelNames();
  ROS_DEBUG_STREAM_NAMED("grasp","Number of links in group " << EE_GROUP << ": " << ee_link_names.size());

  // Robot Interaction - finds the end effector associated with a planning group
  robot_interaction::RobotInteraction robot_interaction( planning_scene_monitor_->getRobotModel() );

  // Decide active end effectors
  robot_interaction.decideActiveEndEffectors(PLANNING_GROUP_NAME);

  // Get active EE
  std::vector<robot_interaction::RobotInteraction::EndEffector>
    active_eef = robot_interaction.getActiveEndEffectors();
  ROS_DEBUG_STREAM_NAMED("grasp","Number of active end effectors: " << active_eef.size());
  if( !active_eef.size() )
  {
    ROS_ERROR_STREAM_NAMED("grasp","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
    return false;
  }

  // Just choose the first end effector TODO: better logic?
  robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

  // -----------------------------------------------------------------------------------------------
  // Get EE link markers for Rviz
  robot_state.getRobotMarkers(marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
  ROS_DEBUG_STREAM_NAMED("grasp","Number of rviz markers in end effector: " << marker_array_.markers.size());

  // Change pose from Eigen to TF
  try{
    tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
  }
  catch(...)
  {
    ROS_ERROR_STREAM_NAMED("grasp","Didn't find link state for " << eef.parent_link);
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

void GraspGenerator::publishEEMarkers(const geometry_msgs::Pose &grasp_pose)
{
  //ROS_INFO_STREAM("Mesh (" << grasp_pose.position.x << ","<< grasp_pose.position.y << ","<< grasp_pose.position.z << ")");

  // -----------------------------------------------------------------------------------------------
  // Make sure EE Marker is loaded
  if( !ee_marker_is_loaded_ )
  {
    ROS_INFO_STREAM_NAMED("grasp","Loading end effector rviz markers");
    if( !loadEEMarker() )
    {
      ROS_WARN_STREAM_NAMED("grasp","Unable to publish EE marker");
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

}

void GraspGenerator::publishSphere(const geometry_msgs::Pose &pose)
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

  marker.lifetime = marker_lifetime_;

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

void GraspGenerator::publishArrow(const geometry_msgs::Pose &pose)
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

  marker.lifetime = marker_lifetime_;

  rviz_marker_pub_.publish( marker );
  ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz
}

void GraspGenerator::publishBlock(const geometry_msgs::Pose &pose, const double& block_size)
{
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
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker.lifetime = marker_lifetime_;

  rviz_marker_pub_.publish( marker );
  ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz
}

random_numbers::RandomNumberGenerator& GraspGenerator::getRandomNumberGenerator()
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

} // namespace
