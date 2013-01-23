/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dave Coleman
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit/kinematics_planner/kinematics_planner.h>


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

  // Action Servers and Clients
  actionlib::SimpleActionServer<clam_block_manipulation::PickPlaceAction> action_server_;
  actionlib::SimpleActionClient<clam_controller::ClamArmAction> clam_arm_client_;
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> movegroup_action_;

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
    //nh_("~"),
    action_server_(name, false),
    clam_arm_client_("clam_arm", true),
    movegroup_action_("move_group", true)
  {

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO("[pick place] Waiting for the clam_arm action server");
    }

    clam_controller::ClamArmGoal clam_arm_goal_; // sent to the clam_arm_client_server

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    while(!movegroup_action_.waitForServer(ros::Duration(4.0))){ // wait for server to start
      ROS_INFO("[pick place] Waiting for the move_group action server");
    }

    // -----------------------------------------------------------------------------------------------
    // Rviz Visualizations
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Duration(0.5).sleep();

    // ---------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&PickPlaceServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&PickPlaceServer::preemptCB, this));
    action_server_.start();
  }

  // Recieve Action Goal Function
  void goalCB()
  {
    ROS_INFO("[pick place] Received goal -----------------------------------------------");

    goal_ = action_server_.acceptNewGoal();
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
    start_pose.position.z = 0.1;

    end_pose.position.x = 0.2;
    end_pose.position.y = 0.1;
    end_pose.position.z = 0.1;

    if( !pickAndPlace(start_pose, end_pose) )
    {
      ROS_ERROR("[pick place] Pick and place failed");
      result_.success = false;
      action_server_.setSucceeded(result_);
    }
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick place] Got goal from topic! %s", goal_->topic.c_str());

    if( !pickAndPlace(msg->poses[0], msg->poses[1]) )
    {
      ROS_ERROR("[pick place] Pick and place failed");
      result_.success = false;
      action_server_.setSucceeded(result_);
    }

    pick_place_sub_.shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO("[pick place] Preempted");
    action_server_.setPreempted();
  }

  bool sendPoseCommand(const geometry_msgs::Pose& pose)
  {
    // -----------------------------------------------------------------------------------------------
    // Move arm
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
    // Visualize goals
    ROS_INFO_STREAM("[pick place] Sending planning goal to MoveGroup for x:" << x << " y:" << y << " z:" << z);
    publishSphere(x, y, z);
    publishMesh(x, y, z + x_offset, qx, qy, qz, qw );


    // -------------------------------------------------------------------------------------------
    // Plan
    movegroup_action_.sendGoal(goal);
    ros::Duration(5.0).sleep();

    if(!movegroup_action_.waitForResult(ros::Duration(5.0)))
    {
      ROS_INFO_STREAM("[pick place] Returned early?");
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[pick place] It worked!");
    }
    else
    {
      ROS_ERROR_STREAM("[pick place] FAILED: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
    }

    return true;
  }


  // Function for testing multiple directions
  double computeCartesianPathWrapper(moveit_msgs::RobotTrajectory &approach_traj_result, const std::string &ik_link, const Eigen::Vector3d &approach_direction,
                                     double desired_approach_distance, double max_step, kinematic_state::KinematicState approach_state, plan_execution::PlanExecution &plan_execution)
  {
    double d_approach =
      approach_state.getJointStateGroup(GROUP_NAME)->computeCartesianPath(approach_traj_result,
                                                                          ik_link,
                                                                          approach_direction,
                                                                          desired_approach_distance,
                                                                          max_step);          // TODO approach_validCallback);
    ROS_WARN("Cartesian path computed! ----------------------------------------------------------");
    ROS_INFO_STREAM("Approach distance: " << d_approach );


    for(int i = 0; i < approach_traj_results.points.size(); ++i)
    {
      approach_traj_results.points[i].time_from_start = (double)i;
    }

    ROS_INFO_STREAM("Approach Trajectory\n" << approach_traj_result);



    ROS_INFO("\n\n\n\n\n\nSleeping...");


    /* execute the planned trajectory */
    ROS_INFO("Executing trajectory");
    plan_execution.getTrajectoryExecutionManager()->clear();
    if(plan_execution.getTrajectoryExecutionManager()->push(approach_traj_result))
    {
      plan_execution.getTrajectoryExecutionManager()->execute();

      /* wait for the trajectory to complete */
      moveit_controller_manager::ExecutionStatus es = plan_execution.getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_INFO("Trajectory execution succeeded");
      else
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO("Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO("Trajectory execution timed out");
          else
            ROS_INFO("Trajectory execution control failed");
    }
    else
    {
      ROS_ERROR("Failed to push trajectory");
      return -1;
    }

   ros::Duration(4.0).sleep();
  }

  // Actually run the action
  bool pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    geometry_msgs::Pose desired_pose = start_pose;

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
    ROS_INFO("[pick place] Sending arm to pre-grasp position ------------------------------------");
    desired_pose.position.z = 0.1;
    if(!sendPoseCommand(desired_pose))
      return false;






    // ---------------------------------------------------------------------------------------------
    // Compute Straight Line Path
    // try to compute a straight line path that arrives at the goal using the specified approach direction

    ROS_INFO("[pick place] Compute Straight Line Path -------------------------------------------");
    /*
      const ManipulationPlanPtr &plan
      plan->possible_goal_states_ is type kinematic_state::KinematicStatePtr



      TODO
      plan->ik_link_name_
      -approach_direction,
      plan->grasp_.desired_approach_distance,
      max_step_,
      approach_validCallback
    */


    /*
      planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
      planning_scene::PlanningScene &scene2 = *psm.getPlanningScene();
      kinematic_state::KinematicState state = scene2.getCurrentState();
      ROS_INFO("\nState info: \n");
      state.printStateInfo();
      ROS_WARN("\n\n\n\n");
    */



    // Planning Scene stuff
    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
    //planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));
    if (planning_scene_monitor->getPlanningScene() && planning_scene_monitor->getPlanningScene()->isConfigured())
    {
      ROS_INFO("Planning scene configured");

      planning_scene_monitor->startWorldGeometryMonitor();
      planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor->startStateMonitor("/joint_states", "/attached_collision_object");

      //planning_scene_monitor->startSceneMonitor();
      //planning_scene_monitor->startStateMonitor();

      ros::Duration(0.1).sleep();
    }
    else
    {
      ROS_ERROR("Planning scene not configured");
      return false;
    }

    std::vector<std::string> missing_joints;

    while( !planning_scene_monitor->getStateMonitor()->haveCompleteState() )
    {
      ros::Duration(0.1).sleep();

      ros::spinOnce();
      ROS_INFO("Waiting for complete state...");

      planning_scene_monitor->getStateMonitor()->haveCompleteState( missing_joints );

      // Show unpublished joints
      for(int i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM("Unpublished joints: " << missing_joints[i]);
    }

    const planning_scene::PlanningScenePtr scene = planning_scene_monitor->getPlanningScene();

    // Create a goal kinematic state (just hardcode grasp position
    kinematic_state::KinematicState approach_state = scene->getCurrentState();
    //approach_state.setToCurrent();
    // TODO: use setFromIK in joint_state_group.h

    // Output state info
    ROS_INFO("\nState info: \n");
    approach_state.printStateInfo();


    const trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor->getKinematicModel()));
    plan_execution::PlanExecution plan_execution(planning_scene_monitor, trajectory_execution_manager);



    // this is the resulting generated trajectory
    moveit_msgs::RobotTrajectory approach_traj_result;

    // End effector parent link
    const std::string &ik_link = "gripper_roll_link"; // eef->getEndEffectorParentGroup().second;
    // TODO: also try for ik_link "gripper_fixed_finger_link"... maybe?

    // Approach direction
    Eigen::Vector3d approach_direction;
    approach_direction << 1, 0, 0; // TODO: test

    // The distance the origin of a robot link needs to travel
    double desired_approach_distance = .025; // 25cm

    // Resolution of trajectory
    double max_step = 0.01; // The maximum distance in Cartesian space between consecutive points on the resulting path

    /*
    // Check for kinematic solver
    if( !approach_state.getJointStateGroup(GROUP_NAME)->getJointModelGroup()->canSetStateFromIK( ik_link ) )
    {
    // Set kinematic solver
    const std::pair<kinematic_model::SolverAllocatorFn, kinematic_model::SolverAllocatorMapFn> &allocators =
    approach_state.getJointStateGroup(GROUP_NAME)->getJointModelGroup()->getSolverAllocators();
    if( allocators.first)
    ROS_INFO("Has IK Solver");
    else
    ROS_INFO("No has IK Solver");
    }
    */

    /** \brief Compute the sequence of joint values that correspond to a Cartesian path. The Cartesian path to be followed is specified
        as a direction of motion (\e direction) for the origin of a robot link (\e link_name).  The link needs to move in a straight
        line, following the specified direction, for the desired \e distance. The resulting joint values are stored in the vector \e states,
        one by one. The maximum distance in Cartesian space between consecutive points on the resulting path is specified by \e max_step.
        If a \e validCallback is specified, this is passed to the internal call to setFromIK(). In case of failure, the computation of the path
        stops and the value returned corresponds to the distance that was computed and for which corresponding states were added to the path.
        At the end of the function call, the state of the group corresponds to the last attempted Cartesian pose

        double computeCartesianPath(moveit_msgs::RobotTrajectory &traj, const std::string &link_name, const Eigen::Vector3d &direction,
        ........................... double distance, double max_step, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn()); */

    ROS_WARN("Preparing to computer cartesian path");


    approach_direction << -1,0,0;

    computeCartesianPathWrapper(approach_traj_result,
                                ik_link,
                                approach_direction,
                                desired_approach_distance,
                                max_step, approach_state, plan_execution);          // TODO approach_validCallback);

    /*
      approach_direction << 1,0,0;

      computeCartesianPathWrapper(approach_traj_result,
      ik_link,
      approach_direction,
      desired_approach_distance,
      max_step, approach_state, plan_execution);          // TODO approach_validCallback);

      approach_direction << 0,1,0;

      computeCartesianPathWrapper(approach_traj_result,
      ik_link,
      approach_direction,
      desired_approach_distance,
      max_step, approach_state, plan_execution);          // TODO approach_validCallback);

      approach_direction << 0,-1,0;

      computeCartesianPathWrapper(approach_traj_result,
      ik_link,
      approach_direction,
      desired_approach_distance,
      max_step, approach_state, plan_execution);          // TODO approach_validCallback);

      approach_direction << 0,0,1;

      computeCartesianPathWrapper(approach_traj_result,
      ik_link,
      approach_direction,
      desired_approach_distance,
      max_step, approach_state, plan_execution);          // TODO approach_validCallback);

      approach_direction << 0,0,-1;

      computeCartesianPathWrapper(approach_traj_result,
      ik_link,
      approach_direction,
      desired_approach_distance,
      max_step, approach_state, plan_execution);          // TODO approach_validCallback);

    */


    ROS_WARN("Done ------------------------------------------------------------------------------");
    /*

    // ---------------------------------------------------------------------------------------------
    // Lower over block
    ROS_INFO("[pick place] Sending arm to grasp position ----------------------------------------");
    desired_pose.position.z -= 0.05; // lower
    //ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendPoseCommand(desired_pose))
    return false;

    // ---------------------------------------------------------------------------------------------
    // Close gripper
    ROS_INFO("[pick place] Closing gripper");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::END_EFFECTOR_CLOSE;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
    ros::Duration(0.1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Move Arm to new location
    ROS_INFO("[pick place] Sending arm to new position ------------------------------------------");
    desired_pose = end_pose;
    desired_pose.position.z = 0.1;
    //ROS_INFO_STREAM("[pick place] Pose: \n" << desired_pose );
    if(!sendPoseCommand(desired_pose))
    return false;

    // ---------------------------------------------------------------------------------------------
    // Open gripper
    ROS_INFO("[pick place] Opening gripper");
    clam_arm_goal_.command = clam_controller::ClamArmGoal::END_EFFECTOR_OPEN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
    ros::Duration(0.1).sleep();

    */
    // ---------------------------------------------------------------------------------------------
    // Reset
    /*
      ROS_INFO("[pick place] Going to home position");
      clam_arm_goal_.command = clam_controller::ClamArmGoal::RESET;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();
    */

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO("[pick place] Finished ------------------------------------------------");
    ROS_INFO(" ");
    action_server_.setSucceeded(result_);
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

