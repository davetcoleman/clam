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
GraspGenerator::GraspGenerator(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                               std::string base_link, bool rviz_verbose, RobotVizToolsPtr rviz_tools,
                               const std::string planning_group ):
  planning_scene_monitor_(planning_scene_monitor),
  base_link_(base_link),
  rviz_verbose_(rviz_verbose),
  rviz_tools_(rviz_tools),
  planning_group_(planning_group),
  nh_("~")
{

  // -----------------------------------------------------------------------------------------------
  // Adding collision objects
  //collision_obj_pub_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 1);

  // ---------------------------------------------------------------------------------------------
  // Check planning scene monitor
  if (planning_scene_monitor_->getPlanningScene())
  {
    /*
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    */

    // Only publish planning scene if we are in verbose mode
    if(rviz_verbose_)
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "dave_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("grasp","Planning scene not configured");
  }

  ROS_INFO_STREAM_NAMED("grasp","GraspGenerator ready.");
}

GraspGenerator::~GraspGenerator()
{
  ROS_INFO_STREAM_NAMED("grasp","unloading GraspGenerator");
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
  if(rviz_verbose_)
  {
    //rviz_tools_->publishBlock(block_pose, BLOCK_SIZE, true);
  }

  // ---------------------------------------------------------------------------------------------
  // Generate grasps

  // Calculate grasps in two axis in both directions
  //generateAxisGrasps( possible_grasps, X_AXIS, DOWN );
  //generateAxisGrasps( possible_grasps, X_AXIS, UP );
  //generateAxisGrasps( possible_grasps, Y_AXIS, DOWN );
  generateAxisGrasps( possible_grasps, Y_AXIS, UP );
  ROS_DEBUG_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  // Visualize initial results
  if(rviz_verbose_)
  {
    //visualizeGrasps(possible_grasps, block_pose);
  }

  // Just test with one for now
  //filterNthGrasp(possible_grasps, 4);


  // Visualize results
  if(rviz_verbose_)
  {
    visualizeGrasps(possible_grasps, block_pose);
  }

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
  gripper_approach.direction.header.frame_id = "/gripper_roll_link";
  gripper_approach.direction.vector.x = 1;
  gripper_approach.direction.vector.y = 0;
  gripper_approach.direction.vector.z = 0; // Approach direction (negative z axis)

  // Create re-usable retreat motion
  manipulation_msgs::GripperTranslation gripper_retreat;
  gripper_retreat.direction.header.stamp = ros::Time::now();
  gripper_retreat.direction.header.frame_id = "/gripper_roll_link";
  gripper_retreat.direction.vector.x = -1;
  gripper_retreat.direction.vector.y = 0;
  gripper_retreat.direction.vector.z = 0; // Retreat direction (pos z axis)
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
  //double angle_resolution = 16.0;
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

    Eigen::Quaterniond quaternion(rotation_matrix);
    grasp_pose.pose.orientation.x = quaternion.x();
    grasp_pose.pose.orientation.y = quaternion.y();
    grasp_pose.pose.orientation.z = quaternion.z();
    grasp_pose.pose.orientation.w = quaternion.w();

    // Calculate the theta1 for next time
    theta1 += M_PI / angle_resolution;

    // ---------------------------------------------------------------------------------------------
    // Create a Grasp message
    manipulation_msgs::Grasp new_grasp;

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

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
    // Convert either:
    // Eigen::Matrix3d rotation_matrix;
    // Eigen::Quaterniond quaternion
    // To:
    // Eigen::Vector3d

    // Method 1
    // Convert from/to
    // tf::Pose grasp_pose.pose.orientation
    // Eigen::Matrix3d
    /*
    Eigen::Quaterniond base_frame_quat;
    base_frame_quat.x() = grasp_pose.pose.orientation.x;
    base_frame_quat.y() = grasp_pose.pose.orientation.y;
    base_frame_quat.z() = grasp_pose.pose.orientation.z;
    base_frame_quat.w() = grasp_pose.pose.orientation.w;
    Eigen::Matrix3d base_rot_matrix = base_frame_quat.matrix();
    Eigen::Vector3d approach_direction = base_rot_matrix.eulerAngles(1,0,0);
    gripper_approach.direction.header.frame_id = "/gripper_roll_link"; //base_link_;
    tf::vectorEigenToMsg(approach_direction, gripper_approach.direction.vector);

    if( grasp_id == 4 )
      ROS_INFO_STREAM_NAMED("grasp", new_grasp.id << ": Approach direction 1: [" << gripper_approach.direction.vector.x << "," <<
                            gripper_approach.direction.vector.y << "," << gripper_approach.direction.vector.z << "]");

    // Method 2

    if( grasp_id == 4 )
      ROS_INFO_STREAM_NAMED("grasp", new_grasp.id << ": Approach direction 2: [" << gripper_approach.direction.vector.x << "," <<
                            gripper_approach.direction.vector.y << "," << gripper_approach.direction.vector.z << "]\n");
    */

    //gripper_approach.desired_distance = .050; // The distance the origin of a robot link needs to travel
    //gripper_approach.min_distance = .025; // half of the desired? Untested.
    gripper_approach.desired_distance = .050; // The distance the origin of a robot link needs to travel
    gripper_approach.min_distance = .025; // half of the desired? Untested.
    new_grasp.approach = gripper_approach;

    // The retreat motion
    new_grasp.retreat = gripper_retreat;

    if( grasp_id == 4 )
      ROS_INFO_STREAM_NAMED("grasp", new_grasp.id << ": Retreat direction 2: [" << gripper_retreat.direction.vector.x << "," <<
                            gripper_retreat.direction.vector.y << "," << gripper_retreat.direction.vector.z << "]\n");

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // an optional list of obstacles that we have semantic information about
    // and that can be touched/pushed/moved in the course of grasping
    //string[] allowed_touch_objects

    // Add to vector
    possible_grasps.push_back(new_grasp);
  }


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
  // how many cores does this computer have and how many do we need?
  int num_threads = boost::thread::hardware_concurrency();
  if( num_threads > possible_grasps.size() )
    num_threads = possible_grasps.size();
  num_threads = 1;

  // -----------------------------------------------------------------------------------------------
  // Get the solver timeout from kinematics.yaml
  double timeout = planning_scene_monitor_->getPlanningScene()->getCurrentState().
    getJointStateGroup(planning_group_)->getDefaultIKTimeout();

  // -----------------------------------------------------------------------------------------------
  // Load kinematic solvers if not already loaded
  if( kin_solvers_.size() != num_threads )
  {
    kin_solvers_.clear();

    boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kin_plugin_loader;
    kin_plugin_loader.reset(new kinematics_plugin_loader::KinematicsPluginLoader());
    kinematics_plugin_loader::KinematicsLoaderFn kin_allocator = kin_plugin_loader->getLoaderFunction();

    const robot_model::JointModelGroup* joint_model_group
      = planning_scene_monitor_->getPlanningScene()->getRobotModel()->getJointModelGroup(planning_group_);

    // Create an ik solver for every thread
    for (int i = 0; i < num_threads; ++i)
    {
      //ROS_INFO_STREAM_NAMED("grasp","Creating ik solver " << i);
      kin_solvers_.push_back(kin_allocator(joint_model_group));
    }
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();
  {

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find those that are kinematically feasible
    std::vector<manipulation_msgs::Grasp> filtered_grasps;

    boost::thread_group bgroup; // create a group of threads
    boost::mutex lock; // used for sharing the same data structures

    ROS_INFO_STREAM_NAMED("grasp", "Filtering possible grasps with " << num_threads << " threads");

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

      IkThreadStruct tc(possible_grasps, filtered_grasps, grasps_id_start, grasps_id_end,
                        kin_solvers_[i], timeout, &lock, i);
      bgroup.create_thread( boost::bind( &GraspGenerator::filterGraspThread, this, tc ) );
    }

    ROS_INFO_STREAM_NAMED("grasp","Waiting to joint threads...");
    bgroup.join_all(); // wait for all threads to finish
    ROS_INFO_STREAM_NAMED("grasp","Done waiting to joint threads...");

    ROS_INFO_STREAM_NAMED("grasp", "Found " << filtered_grasps.size() << " ik solutions out of " <<
                          possible_grasps.size() );

    possible_grasps = filtered_grasps;

  }
  // End Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("grasp","Grasp generator IK grasp filtering benchmark time:");
  std::cout << duration << "\t" << possible_grasps.size() << "\n";

  ROS_INFO_STREAM_NAMED("grasp","Possible grasps filtered to " << possible_grasps.size() << " options.");

  return true;
}

// Thread for checking part of the possible grasps list
void GraspGenerator::filterGraspThread(IkThreadStruct ik_thread_struct)
{
  // Seed state - start at zero
  std::vector<double> ik_seed_state(7); // fill with zeros

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::Pose* ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    ROS_DEBUG_STREAM_NAMED("grasp", "Checking grasp #" << i);

    // Pointer to current pose
    ik_pose = &ik_thread_struct.possible_grasps_[i].grasp_pose.pose;

    // Test it with IK
    ik_thread_struct.kin_solver_->
      searchPositionIK(*ik_pose, ik_seed_state, ik_thread_struct.timeout_, solution, error_code);

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      ROS_INFO_STREAM_NAMED("grasp","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = solution;

      // Copy solution to manipulation_msg so that we can use it later
      // Note: doesn't actually belong here TODO: fix this hack
      ik_thread_struct.possible_grasps_[i].grasp_posture.position = solution;

      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );
      }

      // TODO: is this thread safe? (prob not)
      if(rviz_verbose_)
        rviz_tools_->publishArrow(*ik_pose);
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      //ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose: Timed Out.");
      //std::copy(solution.begin(),solution.end(), std::ostream_iterator<double>(std::cout, "\n"));
    }
    else
      ROS_INFO_STREAM_NAMED("grasp","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }

  ROS_INFO_STREAM_NAMED("grasp","Thread " << ik_thread_struct.thread_id_ << " finished");
}

// Show all grasps in Rviz
void GraspGenerator::visualizeGrasps(const std::vector<manipulation_msgs::Grasp>& possible_grasps,
                                     const geometry_msgs::Pose& block_pose)
{
  if(!rviz_verbose_)
    return; // this function will only work if we have loaded the publishers

  // isRed = true if possible_blocks is empty
  rviz_tools_->publishBlock(block_pose, BLOCK_SIZE, possible_grasps.empty() );

  // Show robot joint positions if available
  if( !possible_grasps.empty() && !possible_grasps[0].grasp_posture.position.empty() )
    rviz_tools_->publishPlanningScene(possible_grasps[0].grasp_posture.position);


  for(std::vector<manipulation_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
      grasp_it < possible_grasps.end(); ++grasp_it)
  {
    geometry_msgs::Pose grasp_pose = grasp_it->grasp_pose.pose;

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose\n" << grasp_pose);
    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose");

    //rviz_tools_->publishSphere(grasp_pose);
    rviz_tools_->publishArrow(grasp_pose);
    rviz_tools_->publishEEMarkers(grasp_pose);

    // Show robot joint positions if available
    //if( !grasp_it->grasp_posture.position.empty() )
    //rviz_tools_->publishPlanningScene(grasp_it->grasp_posture.position);

    //ros::Duration(0.01).sleep();
    ros::Duration(0.1).sleep();
  }
}

} // namespace
