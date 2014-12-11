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
   Desc:   Shell for the MoveIt Pick Place pipeline
*/

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Grasp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ClamArm
#include <clam_msgs/PickPlaceAction.h>
#include <clam_msgs/ClamArmAction.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit_msgs/PickupAction.h>

namespace clam_block_manipulation
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string EE_NAME = "end_effector";
static const std::string EE_GROUP = "gripper_group";
static const std::string EE_LINK_FRAME = "/gripper_roll_link";
static const std::string COLLISION_TOPIC = "/collision_object";

static const double BLOCK_SIZE = 0.04;

// Required for RobotVizTools:
static const std::string PLANNING_GROUP_NAME = "arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";


// Class
class PickPlaceMoveItServer
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher rviz_marker_pub_;
  ros::Publisher collision_obj_pub_;

  // Actions
  actionlib::SimpleActionServer<clam_msgs::PickPlaceAction> action_server_;
  actionlib::SimpleActionClient<moveit_msgs::PickupAction> movegroup_action_;
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_;

  clam_msgs::ClamArmGoal           clam_arm_goal_; // sent to the clam_arm_action_server
  clam_msgs::PickPlaceFeedback     feedback_;
  clam_msgs::PickPlaceResult       result_;
  clam_msgs::PickPlaceGoalConstPtr pick_place_goal_;

  // Save collision object so we can delete it later
  moveit_msgs::CollisionObject chosen_block_object_;
  bool block_published_;

  // Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // Parameters
  std::string base_link_;

  // End Effector Markers
  bool ee_marker_is_loaded_; // determines if we have loaded the marker or not
  visualization_msgs::MarkerArray marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_;
  std::vector<geometry_msgs::Pose> marker_poses_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr rviz_tools_;

public:

  // Constructor
  PickPlaceMoveItServer(const std::string name):
    nh_("~"),
    movegroup_action_("pickup", true),
    clam_arm_client_("clam_arm", true),
    ee_marker_is_loaded_(false),
    block_published_(false),
    action_server_(name, false)
  {
    base_link_ = "/base_link";

    // -----------------------------------------------------------------------------------------------
    // Adding collision objects
    collision_obj_pub_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 1);

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group/Pickup action server
    while(!movegroup_action_.waitForServer(ros::Duration(4.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick_place_moveit","Waiting for the move_group/Pickup action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Connect to ClamArm action server
    while(!clam_arm_client_.waitForServer(ros::Duration(5.0))){ // wait for server to start
      ROS_INFO_STREAM_NAMED("pick_place_moveit","Waiting for the clam_arm action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

    if (planning_scene_monitor_->getPlanningScene())
    {
      //planning_scene_monitor_->startWorldGeometryMonitor();
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("pick_place_moveit","Planning scene not configured");
    }

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    rviz_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_link_, RVIZ_MARKER_TOPIC));

    // ---------------------------------------------------------------------------------------------
    // Register the goal and preempt callbacks
    action_server_.registerGoalCallback(boost::bind(&PickPlaceMoveItServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&PickPlaceMoveItServer::preemptCB, this));
    action_server_.start();

    // Announce state
    ROS_INFO_STREAM_NAMED("pick_place_moveit", "Server ready.");
    ROS_INFO_STREAM_NAMED("pick_place_moveit", "Waiting for pick command...");

    // ---------------------------------------------------------------------------------------------
    // Send home
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Sending home");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    while(!clam_arm_client_.getState().isDone() && ros::ok())
      ros::Duration(0.1).sleep();

    
    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, "clam_arm"))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Send fake command
    fake_goalCB();
  }

  // Destructor
  ~PickPlaceMoveItServer()
  {
  }

  // Action server sends goals here
  void goalCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Received goal -----------------------------------------------");

    pick_place_goal_ = action_server_.acceptNewGoal();
    base_link_ = pick_place_goal_->frame;

    processGoal(pick_place_goal_->pickup_pose, pick_place_goal_->place_pose);
  }

  // Skip perception
  void fake_goalCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Received fake goal ----------------------------------------");

    // Position
    geometry_msgs::Pose start_block_pose;
    geometry_msgs::Pose end_block_pose;

    // Does not work
    start_block_pose.position.x = 0.35;
    start_block_pose.position.y = 0.2;
    start_block_pose.position.z = 0.02;

    // Works - close
    start_block_pose.position.x = 0.2;
    start_block_pose.position.y = 0.0;
    start_block_pose.position.z = 0.02;

    // 3rd try
    start_block_pose.position.x = 0.35;
    start_block_pose.position.y = 0.1;
    start_block_pose.position.z = 0.02;

    nh_.param<double>("/block_pick_place_server/block_x", start_block_pose.position.x, 0.2);
    nh_.param<double>("/block_pick_place_server/block_y", start_block_pose.position.y, 0.0);
    nh_.param<double>("/block_pick_place_server/block_z", start_block_pose.position.z, 0.02);

    ROS_INFO_STREAM_NAMED("pick_place_moveit","start block is \n" << start_block_pose.position);


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
      ROS_ERROR_STREAM_NAMED("pick_place_moveit","Pick and place failed");

      if(action_server_.isActive()) // Make sure we haven't sent a fake goal
      {
        // Report failure
        result_.success = false;
        action_server_.setSucceeded(result_);
      }
    }
    else
    {
      if(action_server_.isActive()) // Make sure we haven't sent a fake goal
      {
        // Report success
        result_.success = true;
        action_server_.setSucceeded(result_);
      }
    }

    // TODO: remove
    ros::shutdown();
  }

  // Cancel the action
  void preemptCB()
  {
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Preempted");
    action_server_.setPreempted();
  }

  // Execute series of tasks for pick/place
  bool pickAndPlace(const geometry_msgs::Pose& start_block_pose, const geometry_msgs::Pose& end_block_pose)
  {
    ROS_INFO_STREAM_NAMED("pick_place","Pick and place started");

    // ---------------------------------------------------------------------------------------------
    // Visualize the two blocks
    rviz_tools_->publishBlock(start_block_pose);
    rviz_tools_->publishBlock(end_block_pose);

    // ---------------------------------------------------------------------------------------------
    // Generate graps
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Generating grasps for pick and place");

    bool rviz_verbose = true;
    moveit_simple_grasps::SimpleGrasps grasp_generator(rviz_tools_);

    // Pick grasp
    std::vector<moveit_msgs::Grasp> possible_grasps;
    grasp_generator.generateBlockGrasps( start_block_pose, grasp_data_, possible_grasps );

    // Filter grasp poses
    //moveit_simple_grasps::GraspFilter grasp_filter( planning_scene_monitor_->getPlanningScene()->getCurrentState() ...
    //if( !grasp_generator.filterGrasps( possible_grasps ) )
    //return false;


    // Send pick command to move_group
    executeGrasps(possible_grasps, start_block_pose);


    return true;
  }


  void createCollisionObject(const geometry_msgs::Pose& block_pose, moveit_msgs::CollisionObject& block_object)
  {
    if( block_published_ )
    {
      return; // only publish the block once!
    }

    ROS_INFO_STREAM_NAMED("pick_place_moveit","Creating the collision object");
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
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Collision object published for addition");
  }

  // *Requires that the object already be created
  void deleteCollisionObject(moveit_msgs::CollisionObject& block_object)
  {
    // Operation to be performed
    block_object.operation = moveit_msgs::CollisionObject::REMOVE;

    // Send the object
    block_published_ = false;
    collision_obj_pub_.publish(block_object);
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Collision object published for removal");
  }

  bool executeGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                     const geometry_msgs::Pose& block_pose)
  {
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Creating Pickup Goal");


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

    // which end-effector to be used for pickup (ideally descpending from the group above)
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

    //ROS_INFO_STREAM_NAMED("pick_place_moveit","Pause");
    //ros::Duration(5.0).sleep();

    // ---------------------------------------------------------------------------------------------
    // Send the grasp to move_group/Pickup
    ROS_INFO_STREAM_NAMED("pick_place_moveit","Sending pick action to move_group/Pickup");

    movegroup_action_.sendGoal(goal);

    if(!movegroup_action_.waitForResult(ros::Duration(20.0)))
    {
      ROS_INFO_STREAM_NAMED("pick_place_moveit","Returned early?");
      return false;
    }
    if (movegroup_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("pick_place_moveit","Plan successful!");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pick_place_moveit","FAILED: " << movegroup_action_.getState().toString() << ": " << movegroup_action_.getState().getText());
      return false;
    }

    return true;
  }


}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_pick_place_moveit_server");

  clam_block_manipulation::PickPlaceMoveItServer server("pick_place_moveit");

  // Allow the action server to recieve and send ros messages
  //  ros::spin(); // keep the action server alive
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

