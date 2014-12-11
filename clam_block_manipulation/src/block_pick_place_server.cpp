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
   Desc:   Controls the movement of the arm, planning, trajectories, etc
*/

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ClamArm
#include <clam_msgs/PickPlaceAction.h>
#include <clam_msgs/ClamArmAction.h>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Grasp generation
#include <moveit_simple_grasps/grasp_generator.h>
#include <moveit_simple_grasps/robot_viz_tools.h> // simple tool for showing grasps

namespace clam_block_manipulation
{

// Static const vars
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_LINK = "gripper_roll_link";
static const double PREGRASP_Z_HEIGHT = 0.09;

// Required for RobotVizTools:
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string EE_GROUP = "gripper_group";

// Class
class BlockPickPlaceServer
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Action Servers and Clients
  actionlib::SimpleActionServer<clam_msgs::PickPlaceAction> action_server_;
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_;
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> movegroup_action_;

  // Action messages
  clam_msgs::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server
  clam_msgs::PickPlaceFeedback     feedback_;
  clam_msgs::PickPlaceResult       result_;
  clam_msgs::PickPlaceGoalConstPtr pick_place_goal_;

  // MoveIt Components
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  // Parameters from goal
  std::string base_link_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr rviz_tools_;

public:

  // Constructor
  BlockPickPlaceServer(const std::string name) :
    action_server_(name, false),
    clam_arm_client_("clam_arm", true),
    movegroup_action_("move_group", true)
  {

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick place","Waiting for the clam_arm action server");
    }

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    while(!movegroup_action_.waitForServer(ros::Duration(4.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick place","Waiting for the move_group action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    tf_.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_));

    // ---------------------------------------------------------------------------------------------
    // Check planning scene monitor
    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Planning scene not configured");
    }

    // ---------------------------------------------------------------------------------------------
    // Create a trajectory execution manager
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
                                        (planning_scene_monitor_->getRobotModel()));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));

    // ---------------------------------------------------------------------------------------------
    // Wait for complete state to be recieved
    ros::Duration(0.25).sleep();

    std::vector<std::string> missing_joints;
    while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() )
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      ROS_INFO_STREAM_NAMED("pick place","Waiting for complete state...");

      // Show unpublished joints
      planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
      for(int i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("pick_place","Unpublished joints: " << missing_joints[i]);
    }

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    rviz_tools_.reset(new moveit_simple_grasps::RobotVizTools(RVIZ_MARKER_TOPIC, EE_GROUP, PLANNING_GROUP_NAME, base_link_, planning_scene_monitor_));

    // ---------------------------------------------------------------------------------------------
    // Register the goal and preempt callbacks
    action_server_.registerGoalCallback(boost::bind(&BlockPickPlaceServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&BlockPickPlaceServer::preemptCB, this));
    action_server_.start();

    // Announce state
    ROS_INFO_STREAM_NAMED("pick_place", "Server ready.");
    ROS_INFO_STREAM_NAMED("pick_place", "Waiting for pick command...");

    // ---------------------------------------------------------------------------------------------
    // Send fake command
    fake_goalCB();
  }

  // Action server sends goals here
  void goalCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Received goal -----------------------------------------------");

    pick_place_goal_ = action_server_.acceptNewGoal();
    base_link_ = pick_place_goal_->frame;

    processGoal(pick_place_goal_->pickup_pose, pick_place_goal_->place_pose);
  }

  // Skip perception
  void fake_goalCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Received fake goal ----------------------------------------");

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

    // Fill goal
    base_link_ = "base_link";

    processGoal(start_block_pose, end_block_pose);
  }

  void processGoal(const geometry_msgs::Pose& start_block_pose, const geometry_msgs::Pose& end_block_pose )
  {
    // Change the goal constraints on the servos to be less strict, so that the controllers don't die. this is a hack
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/elbow_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/shoulder_pan_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/gripper_roll_joint/goal", 2); // originally it was 0.45
    nh_.setParam("/clam_trajectory_controller/joint_trajectory_action_node/constraints/wrist_pitch_joint/goal", 2); // originally it was 0.45

    if( !pickAndPlace(start_block_pose, end_block_pose) )
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Pick and place failed");

      // Report failure
      result_.success = false;
      if(action_server_.isActive()) // Make sure we haven't sent a fake goal
        action_server_.setSucceeded(result_);
    }
    else
    {
      // Report success
      result_.success = true;
      action_server_.setSucceeded(result_);
    }

    // TODO: remove
    ros::shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Preempted");
    action_server_.setPreempted();
  }

  /*
    bool sendGraspPoseCommand(const geometry_msgs::Pose& pose)
    {
    geometry_msgs::Pose goal_pose;
    goal_pose.position = pose.position;
    // Set a straight verticle orientation
    goal_pose.orientation.x = 0.00;
    goal_pose.orientation.y = 0.710502;
    goal_pose.orientation.z = -0.01755;
    goal_pose.orientation.w = 0.70346;

    // Compensates for gripper size
    double x_offset = 0.15;
    return sendPoseCommand( goal_pose, x_offset );
    }
  */

  // Execute series of tasks for pick/place
  bool pickAndPlace(const geometry_msgs::Pose& start_block_pose, const geometry_msgs::Pose& end_block_pose)
  {
    ROS_INFO_STREAM_NAMED("pick_place","Pick and place started");

    // ---------------------------------------------------------------------------------------------
    // Generate graps
    ROS_INFO_STREAM_NAMED("pick_place","Generating grasps for pick and place");
    bool rviz_verbose = true;
    moveit_simple_grasps::GraspGenerator grasp_generator( planning_scene_monitor_, base_link_, rviz_verbose,
                                                           PLANNING_GROUP_NAME);

    // Pick grasp
    std::vector<moveit_msgs::Grasp> possible_grasps;
    grasp_generator.generateGrasps( start_block_pose, possible_grasps );

    moveit_msgs::Grasp pick_grasp;
    // Filter grasp poses
    //moveit_simple_grasps::GraspFilter grasp_filter( planning_scene_monitor_->getPlanningScene()->getCurrentState() ...
    //if( !grasp_generator.filterGrasps( possible_grasps ) )
    //return false;
    //grasp_generator.chooseBestGrasp( possible_grasps, pick_grasp );
    // TODO
    zebra
    geometry_msgs::Pose pick_pose = pick_grasp.grasp_pose.pose;


    return false; // TODO: temp





    // Place grasp
    possible_grasps.clear();
    grasp_generator.generateGrasps( end_block_pose, possible_grasps );

    moveit_msgs::Grasp place_grasp;
    grasp_generator.chooseBestGrasp( possible_grasps, place_grasp );
    geometry_msgs::Pose place_pose = place_grasp.grasp_pose.pose;


    //ROS_INFO("temp kill");
    //return false; // temp

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO_STREAM_NAMED("pick_place","Opening gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Hover over block
    ROS_INFO_STREAM_NAMED("pick_place","Sending arm to pre-grasp position ----------------------------------");
    pick_pose.position.z = PREGRASP_Z_HEIGHT; // a good number for hovering

    double x_offset = 0.15;
    if(!sendPoseCommand(pick_pose, x_offset))
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to go to pre-grasp position");
      return false;
    }

    // ---------------------------------------------------------------------------------------------
    // Lower over block
    // try to compute a straight line path that arrives at the goal using the specified approach direction
    ROS_INFO_STREAM_NAMED("pick_place","Lowering over block -------------------------------------------");
    Eigen::Vector3d approach_direction; // Approach direction (negative z axis)
    approach_direction << 0, 0, -1;
    double desired_approach_distance = .050; // The distance the origin of a robot link needs to travel

    if( !computeStraightLinePath(approach_direction, desired_approach_distance) )
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to follow straight line path");
      return false;
    }
    ros::Duration(0.5).sleep();

    // ---------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO_STREAM_NAMED("pick_place","Closing gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_CLOSE;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Lifting block
    // try to compute a straight line path that arrives at the goal using the specified approach direction
    ROS_INFO_STREAM_NAMED("pick_place","Lifting block -------------------------------------------");

    approach_direction << 0, 0, 1; // Approach direction (positive z axis)
    desired_approach_distance = .050; // The distance the origin of a robot link needs to travel

    if( !computeStraightLinePath(approach_direction, desired_approach_distance) )
    {xo
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to follow straight line path");
      return false;
    }
    ros::Duration(0.5).sleep();


    // ---------------------------------------------------------------------------------------------
    // Move Arm to new location
    ROS_INFO_STREAM_NAMED("pick_place","Sending arm to new position ------------------------------------------");
    place_pose.position.z = PREGRASP_Z_HEIGHT;
    //ROS_INFO_STREAM_NAMED("pick_place","Pose: \n" << place_pose );
    if(!sendPoseCommand(place_pose, x_offset))
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to go to goal position");
      return false;
    }
    ros::Duration(1.0).sleep();


    // ---------------------------------------------------------------------------------------------
    // Lower block
    // try to compute a straight line path that arrives at the goal using the specified approach direction
    ROS_INFO_STREAM_NAMED("pick_place","Lifting block -------------------------------------------");

    approach_direction << 0, 0, -1; // Approach direction (negative z axis)
    desired_approach_distance = .040; // The distance the origin of a robot link needs to travel

    if( !computeStraightLinePath(approach_direction, desired_approach_distance) )
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to follow straight line path");
      return false;
    }
    ros::Duration(0.5).sleep();

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO_STREAM_NAMED("pick_place","Opening gripper");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();
    ros::Duration(2.0).sleep();

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO_STREAM_NAMED("pick_place","Finished ------------------------------------------------");
    ROS_INFO_STREAM_NAMED("pick_place"," ");

    return true;
  }

  // Moves the arm to a specified pose
  bool sendPoseCommand(const geometry_msgs::Pose& pose, double x_offset = 0.0)
  {
    // -----------------------------------------------------------------------------------------------
    // Make a stamped version of the pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = pose;

    // -----------------------------------------------------------------------------------------------
    // Crate move_group goal
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = PLANNING_GROUP_NAME;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;

    // -------------------------------------------------------------------------------------------
    // Create goal state
    goal_pose.header.frame_id = base_link_;
    double tolerance_pose = 1e-4; // default: 1e-3... meters
    double tolerance_angle = 1e-2; // default 1e-2... radians
    moveit_msgs::Constraints goal_constraint0 =
      kinematic_constraints::constructGoalConstraints(EE_LINK, goal_pose,
                                                      tolerance_pose, tolerance_angle);

    ROS_INFO_STREAM_NAMED("pick_place","Goal pose with x_offset of: " << x_offset << "\n" << goal_pose);

    // Create offset constraint
    goal_constraint0.position_constraints[0].target_point_offset.x = x_offset;
    goal_constraint0.position_constraints[0].target_point_offset.y = 0.0;
    goal_constraint0.position_constraints[0].target_point_offset.z = 0.0;

    // Add offset constraint
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = goal_constraint0;

    // -------------------------------------------------------------------------------------------
    // Visualize goals in rviz
    ROS_INFO_STREAM_NAMED("pick_place","Sending planning goal to MoveGroup for x:" << goal_pose.pose.position.x <<
                          " y:" << goal_pose.pose.position.y << " z:" << goal_pose.pose.position.z);
    rviz_tools_->publishSphere(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    rviz_tools_->publishMesh(goal_pose.pose.position.x,
                             goal_pose.pose.position.y,
                             goal_pose.pose.position.z + x_offset,
                             goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                             goal_pose.pose.orientation.z, goal_pose.pose.orientation.w );

    // -------------------------------------------------------------------------------------------
    // Plan
    feedback_.status = "Sending goal to move_group action server";
    if(action_server_.isActive()) // Make sure we haven't sent a fake goal
      action_server_.publishFeedback(feedback_);

    movegroup_action_.sendGoal(goal);
    ros::Duration(5.0).sleep();

    if(!movegroup_action_.waitForResult(ros::Duration(5.0)))
    {
      ROS_INFO_STREAM_NAMED("pick_place","Returned early?");
      return false;
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("pick_place","Plan successful!");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place","move_group failed: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
      return false;
    }

    return true;
  }

  /* Function for testing multiple directions
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, double desired_approach_distance )


  {
    // ---------------------------------------------------------------------------------------------
    // Get planning scene
    const planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState approach_state = planning_scene->getCurrentState();

    // Output state info
    approach_state.printStateInfo();
    approach_state.printTransforms();



    // ---------------------------------------------------------------------------------------------
    // Settings for computeCartesianPath

    // End effector parent link
    const std::string &ik_link = EE_LINK; // eef->getEndEffectorParentGroup().second;

    // Resolution of trajectory
    double max_step = 0.001; // The maximum distance in Cartesian space between consecutive points on the resulting path

    // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
    double jump_threshold = 0.0; // disabled

    // ---------------------------------------------------------------------------------------------
    // Check for kinematic solver
    if( !approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->getJointModelGroup()->canSetStateFromIK( ik_link ) )
    {
      // Set kinematic solver
      const std::pair<robot_model::SolverAllocatorFn, robot_model::SolverAllocatorMapFn> &allocators =
        approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->getJointModelGroup()->getSolverAllocators();
      if( !allocators.first)
        ROS_ERROR_STREAM_NAMED("pick_place","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");
    }

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path
    ROS_INFO_STREAM_NAMED("pick_place","Preparing to computer cartesian path");


    /** \brief Compute the sequence of joint values that correspond to a Cartesian path.

        The Cartesian path to be followed is specified as a direction of motion (\e direction, unit vector) for the origin of a robot
        link (\e link_name). The direction is assumed to be either in a global reference frame or in the local reference frame of the
        link. In the latter case (\e global_reference_frame is true) the \e direction is rotated accordingly. The link needs to move in a
        straight line, following the specified direction, for the desired \e distance. The resulting joint values are stored in
        the vector \e states, one by one. The maximum distance in Cartesian space between consecutive points on the resulting path
        is specified by \e max_step.  If a \e validCallback is specified, this is passed to the internal call to|
        setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to the distance that
        was computed and for which corresponding states were added to the path.  At the end of the function call, the state of the
        group corresponds to the last attempted Cartesian pose.  During the computation of the trajectory, it is sometimes prefered if
        consecutive joint values do not 'jump' by a large amount in joint space, even if the Cartesian distance between the
        corresponding points is as expected. To account for this, the \e jump_threshold parameter is provided.  As the joint values
        corresponding to the Cartesian path are computed, distances in joint space between consecutive points are also computed. Once
        the sequence of joint values is computed, the average distance between consecutive points (in joint space) is also computed. It
        is then verified that none of the computed distances is above the average distance by a factor larger than \e jump_threshold. If
        a point in joint is found such that it is further away than the previous one by more than average_consecutive_distance * \e jump_threshold,
        that is considered a failure and the returned path is truncated up to just before the jump. The jump detection can be disabled
        by settin \e jump_threshold to 0.0
        double computeCartesianPath(std::vector<boost::shared_ptr<RobotState> > &traj, const std::string &link_name, const Eigen::Vector3d &direction, bool global_reference_frame,
        ............................double distance, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn());
    */

    //moveit_msgs::RobotTrajectory approach_traj_result; // create resulting generated trajectory (result)
    std::vector<robot_state::RobotStatePtr> approach_traj_result; // create resulting generated trajectory (result)
    //    std::vector<boost::shared_ptr<robot_state::RobotState> > approach_traj_result; // create resulting generated trajectory (result)

    double d_approach =
      approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->computeCartesianPath(approach_traj_result,
                                                                                   ik_link,                   // link name
                                                                                   approach_direction,
                                                                                   true,                      // direction is in global reference frame
                                                                                   desired_approach_distance,
                                                                                   max_step,
                                                                                   jump_threshold
                                                                                   // TODO approach_validCallback
                                                                                   );

    //    double robot_state::JointStateGroup::computeCartesianPath(std::vector<boost::shared_ptr<robot_state::RobotState> >&, const string&, const Vector3d&, bool, double, double, double, const StateValidityCallbackFn&)

    ROS_INFO_STREAM("Approach distance: " << d_approach );
    if( d_approach == 0 )
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to computer cartesian path: distance is 0");
      return false;
    }

    // -----------------------------------------------------------------------------------------------
    // Get current RobotState  (in order to specify all joints not in approach_traj_result)
    robot_state::RobotState this_robot_state = planning_scene->getCurrentState();
    //    robot_state::kinematicStateToRobotState( planning_scene->getCurrentState(), this_robot_state );

    // -----------------------------------------------------------------------------------------------
    // Smooth the path and add velocities/accelerations

    trajectory_processing::IterativeParabolicTimeParameterization iterative_smoother;
    trajectory_msgs::JointTrajectory trajectory_out;

    // Get the joint limits of planning group
    const robot_model::JointModelGroup *joint_model_group =
      planning_scene->getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME);
    const std::vector<moveit_msgs::JointLimits> &joint_limits = joint_model_group->getVariableLimits();


    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr approach_traj(new robot_trajectory::RobotTrajectory(planning_scene->getRobotModel(), PLANNING_GROUP_NAME));
    for (std::size_t k = 0 ; k < approach_traj_result.size() ; ++k)
      approach_traj->addSuffixWayPoint(approach_traj_result[k], 0.0);

    // Perform iterative parabolic smoothing
    iterative_smoother.computeTimeStamps( *approach_traj );
    /*                                         approach_traj,
                                               trajectory_out,
                                               joint_limits,
                                               this_robot_state // start_state
                                               );
    */

    // Copy results to robot trajectory message:
    //    approach_traj_result = approach_traj;

    ROS_INFO_STREAM("New trajectory\n" << approach_traj);

    // -----------------------------------------------------------------------------------------------
    // Display the path in rviz

    // Create publisher
    ros::Publisher display_path_publisher_;
    display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10, true);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Create the message
    moveit_msgs::DisplayTrajectory rviz_display;
    rviz_display.model_id = planning_scene->getRobotModel()->getName();
    //    rviz_display.trajectory_start = this_robot_state;
    //    rviz_display.trajectory.resize(1, approach_traj_result);

    robot_state::robotStateToRobotStateMsg(approach_traj->getFirstWayPoint(), rviz_display.trajectory_start);
    rviz_display.trajectory.resize(1);
    approach_traj->getRobotTrajectoryMsg(rviz_display.trajectory[0]);


    // Publish message
    display_path_publisher_.publish(rviz_display);
    ROS_INFO_STREAM_NAMED("pick_place","Sent display trajectory message");

    ROS_INFO_STREAM_NAMED("pick_place","Sleeping 1...\n\n");
    ros::Duration(1.0).sleep();

    // -----------------------------------------------------------------------------------------------
    // Execute the planned trajectory
    ROS_INFO_STREAM_NAMED("pick_place","Executing trajectory");

    // Convert trajectory to a message
    moveit_msgs::RobotTrajectory traj_msg;
    approach_traj->getRobotTrajectoryMsg(traj_msg);

    plan_execution_->getTrajectoryExecutionManager()->clear();
    if(plan_execution_->getTrajectoryExecutionManager()->push(traj_msg))
    {
      plan_execution_->getTrajectoryExecutionManager()->execute();

      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_INFO_STREAM_NAMED("pick_place","Trajectory execution succeeded");
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("pick_place","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO_STREAM_NAMED("pick_place","Trajectory execution timed out");
          else
            ROS_INFO_STREAM_NAMED("pick_place","Trajectory execution control failed");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place","Failed to push trajectory");
      return false;
    }

    return true;
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_pick_place_server");

  clam_block_manipulation::BlockPickPlaceServer server("pick_place");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::spin(); // keep the action server alive

  return 0;
}

