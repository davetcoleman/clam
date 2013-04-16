/*
  Copyright (c) 2011, Antons Rebguns <email>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of the <organization> nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Standard
#include <sstream>
#include <string>
#include <vector>
#include <XmlRpcValue.h>

// Dynamixel Low Level
#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>

// Dynamixel Controllers
#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/multi_joint_controller.h>
#include <dynamixel_hardware_interface/joint_trajectory_action_controller.h>

// Services
#include <dynamixel_hardware_interface/SetComplianceMargin.h>
#include <dynamixel_hardware_interface/SetComplianceSlope.h>

// Messages
#include <dynamixel_hardware_interface/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting package file path
#include <pluginlib/class_list_macros.h>

// Boost
#include <boost/filesystem.hpp>  // for error logging to file

// TODO: remove. For recording data
#include <iostream>
#include <fstream>

PLUGINLIB_DECLARE_CLASS(dynamixel_hardware_interface,
                        JointTrajectoryActionController,
                        controller::JointTrajectoryActionController,
                        controller::MultiJointController)

namespace controller
{
// TODO: lower this const:
static const double ACCEPTABLE_BOUND = 0.05; // amount two positions can vary without being considered different positions.
static const bool USE_ERROR_OUTPUT_LOG = false; // during trajectory execution, log the position and velcoty error


JointTrajectoryActionController::JointTrajectoryActionController()
{
  terminate_ = false;
}

JointTrajectoryActionController::~JointTrajectoryActionController()
{
}

bool JointTrajectoryActionController::initialize(std::string name,
                                                 std::vector<boost::shared_ptr<controller::SingleJointController> > deps)
{
  // Load the multi joint controller that this class inherits from. This loads the list of joint_names_
  if (!MultiJointController::initialize(name, deps))
  {
    return false;
  }

  update_rate_ = 1000;
  state_update_rate_ = 50;

  const std::string prefix = "joint_trajectory_action_node/constraints/";

  c_nh_.param<double>(prefix + "goal_time", goal_time_constraint_, 0.0);
  c_nh_.param<double>(prefix + "stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);
  c_nh_.param<double>("joint_trajectory_action_node/min_velocity", min_velocity_, 0.1);

  goal_constraints_.resize(num_joints_);
  trajectory_constraints_.resize(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i)
  {
    c_nh_.param<double>(prefix + joint_names_[i] + "/goal", goal_constraints_[i], -1.0);
    c_nh_.param<double>(prefix + joint_names_[i] + "/trajectory", trajectory_constraints_[i], -1.0);
  }

  // Setup/resize feedback message
  feedback_msg_.joint_names = joint_names_;
  feedback_msg_.desired.positions.resize(num_joints_);
  feedback_msg_.desired.velocities.resize(num_joints_);
  feedback_msg_.desired.accelerations.resize(num_joints_);
  feedback_msg_.actual.positions.resize(num_joints_);
  feedback_msg_.actual.velocities.resize(num_joints_);
  feedback_msg_.actual.accelerations.resize(num_joints_);
  feedback_msg_.error.positions.resize(num_joints_);
  feedback_msg_.error.velocities.resize(num_joints_);
  feedback_msg_.error.accelerations.resize(num_joints_);

  return true;
}

void JointTrajectoryActionController::start()
{
  command_sub_ = c_nh_.subscribe("command", 50, &JointTrajectoryActionController::processCommand, this);
  state_pub_ = c_nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("state", 50);

  action_server_.reset(new FJTAS(c_nh_, "follow_joint_trajectory",
                                 boost::bind(&JointTrajectoryActionController::processFollowTrajectory, this, _1),
                                 false));
  action_server_->start();
  feedback_thread_ = new boost::thread(boost::bind(&JointTrajectoryActionController::updateState, this));
}

void JointTrajectoryActionController::stop()
{
  {
    boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
    terminate_ = true;
  }

  feedback_thread_->join();
  delete feedback_thread_;

  command_sub_.shutdown();
  state_pub_.shutdown();
  action_server_->shutdown();
}

void JointTrajectoryActionController::processCommand(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  if (action_server_->isActive())
  {
    action_server_->setPreempted();
  }

  while (action_server_->isActive())
  {
    ros::Duration(0.01).sleep();
  }

  processTrajectory(*msg, false);
}

// This is what MoveIt is sending out
void JointTrajectoryActionController::processFollowTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  ROS_INFO("before process traj");
  processTrajectory(goal->trajectory, true);
}

void JointTrajectoryActionController::updateState()
{
  ros::Rate rate(state_update_rate_);

  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (terminate_) { break; }
    }

    feedback_msg_.header.stamp = ros::Time::now();

    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      const dynamixel_hardware_interface::JointState* state = joint_states_[joint_names_[j]];
      feedback_msg_.desired.positions[j] = state->target_position;
      feedback_msg_.desired.velocities[j] = std::abs(state->target_velocity);
      feedback_msg_.actual.positions[j] = state->position;
      feedback_msg_.actual.velocities[j] = std::abs(state->velocity);
      feedback_msg_.error.positions[j] = feedback_msg_.actual.positions[j] - feedback_msg_.desired.positions[j];
      feedback_msg_.error.velocities[j] = feedback_msg_.actual.velocities[j] - feedback_msg_.desired.velocities[j];
    }

    state_pub_.publish(feedback_msg_);

    rate.sleep();
  }
}

// Error check the trajectory then send it to the controllers
void JointTrajectoryActionController::processTrajectory(const trajectory_msgs::JointTrajectory& traj_msg,
                                                        bool is_action)
{
  ROS_INFO("inside process Trajectory");
  control_msgs::FollowJointTrajectoryResult traj_result;
  std::string error_msg;

  int num_points = traj_msg.points.size();

  ROS_DEBUG("Received trajectory with %d points", num_points);

  // Maps from an index in joint_names_ to an index in the JointTrajectory msg "traj_msg"
  std::vector<int> lookup(num_joints_, -1);
  ROS_INFO("lookup");

  // Check that all the joints in the trajectory exist in this multiDOF controller
  for (size_t j = 0; j < num_joints_; ++j)
  {
    for (size_t k = 0; k < traj_msg.joint_names.size(); ++k)
    {
      if (traj_msg.joint_names[k] == joint_names_[j])
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      traj_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      error_msg = "Unable to locate joint " + joint_names_[j] + " in the commanded trajectory";
      ROS_ERROR("%s", error_msg.c_str());
      if (is_action)
      {
        action_server_->setAborted(traj_result, error_msg.c_str());
      }
      return;
    }
  }
  ROS_INFO("after for");

  // Check for initial position
  if (traj_msg.points[0].positions.empty())
  {
    traj_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    error_msg = "First point of trajectory has no positions";
    ROS_ERROR("%s", error_msg.c_str());
    if (is_action)
    {
      action_server_->setAborted(traj_result, error_msg);
    }
    return;
  }

  ROS_INFO("after initial");

  // Find out the duration of each segment in the trajectory
  std::vector<double> durations;
  durations.resize(num_points);

  ROS_INFO("after initial");
  durations[0] = traj_msg.points[0].time_from_start.toSec();
  ROS_INFO("after initial");
  double trajectory_duration = durations[0];
  ROS_INFO("after initial");
  for (int i = 1; i < num_points; ++i)
  {
    durations[i] = (traj_msg.points[i].time_from_start - traj_msg.points[i-1].time_from_start).toSec();
    trajectory_duration += durations[i];
    ROS_DEBUG("tpi: %f, tpi-1: %f", traj_msg.points[i].time_from_start.toSec(), traj_msg.points[i-1].time_from_start.toSec());
    ROS_DEBUG("i: %d, duration: %f, total: %f", i, durations[i], trajectory_duration);
  }


  // Loop through each trajectory point, do error checking and create a corresponding "segment"
  std::vector<Segment> trajectory;
  ros::Time time = ros::Time::now() + ros::Duration(0.01);

  for (int i = 0; i < num_points; ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint point = traj_msg.points[i];
    Segment seg;

    // Decide segments start time
    if (traj_msg.header.stamp == ros::Time(0.0))
    {
      seg.start_time = (time + point.time_from_start).toSec() - durations[i];
    }
    else
    {
      seg.start_time = (traj_msg.header.stamp + point.time_from_start).toSec() - durations[i];
    }

    seg.duration = durations[i];

    ROS_DEBUG("Checking velocity");
    // Checks that the incoming segment has the right number of velocity elements
    if (!point.velocities.empty() && point.velocities.size() != num_joints_)
    {
      traj_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      error_msg = "Command point " + boost::lexical_cast<std::string>(i) + " has " +
        boost::lexical_cast<std::string>(point.velocities.size()) +
        " elements for the velocities, expecting " + boost::lexical_cast<std::string>(num_joints_);
      ROS_ERROR("%s", error_msg.c_str());
      if (is_action)
      {
        action_server_->setAborted(traj_result, error_msg);
      }
      return;
    }
    ROS_DEBUG("Done checking velocity");

    // Checks that the incoming segment has the right number of position elements
    if (!point.positions.empty() && point.positions.size() != num_joints_)
    {
      traj_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      error_msg = "Command point " + boost::lexical_cast<std::string>(i) + " has " +
        boost::lexical_cast<std::string>(point.positions.size()) +
        " elements for the positions, expecting " + boost::lexical_cast<std::string>(num_joints_);
      ROS_ERROR("%s", error_msg.c_str());
      if (is_action)
      {
        action_server_->setAborted(traj_result, error_msg);
      }
      return;
    }

    ROS_INFO("Copying velocities and positions to new datastructure");

    // Error Check for trajectory messages that don't have any velocities included
    if(point.velocities.size() == 0)
    {
      error_msg = "No velocities included in the trajectory method!";
      ROS_ERROR("%s", error_msg.c_str());
      if (is_action)
      {
        action_server_->setAborted(traj_result, error_msg);
      }
      return;
    }

    // Create a new segment datastructure
    seg.velocities.resize(num_joints_);
    seg.positions.resize(num_joints_);
    for (size_t j = 0; j < num_joints_; ++j)
    {
      seg.velocities[j] = point.velocities[lookup[j]];
      seg.positions[j] = point.positions[lookup[j]];
    }
    trajectory.push_back(seg);
  }


  // Check if this trajectory goal is already fullfilled by robot's current position
  bool outside_bounds = false; // flag for remembing if a different position was found
  Segment* last_segment = &trajectory[trajectory.size()-1];

  for( std::size_t i = 0; i < last_segment->positions.size(); ++i)
  {

    std::string joint_name = joint_names_[i];

    ROS_DEBUG_STREAM("Checking for similarity on joint " << joint_name << " with real position " << joint_states_[ joint_name ]->position);
    ROS_DEBUG_STREAM("    Iterator id = " << i << " size " << last_segment->positions.size() << "    Goal position: " << last_segment->positions[i] );

    // Test if outside acceptable bounds, meaning we should continue trajectory as normal
    if( last_segment->positions[i] > (joint_states_[ joint_name ]->position + ACCEPTABLE_BOUND) ||
        last_segment->positions[i] < (joint_states_[ joint_name ]->position - ACCEPTABLE_BOUND) )
    {
      outside_bounds = true;
      break;
    }
  }
  // Check if all the states were inside the position bounds
  if( !outside_bounds )
  {
    // We can exit trajectory
    traj_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    error_msg = "Trajectory execution skipped because goal is same as current state";
    ROS_INFO("%s", error_msg.c_str());
    action_server_->setSucceeded(traj_result, error_msg);
    return;
  }
    ROS_INFO("6");

  // Set the compliance margin and slope
  // TODO: move this to configuration file
  int traj_compliance_margin = 1;
  int traj_compliance_slope = 10; //20
  MultiJointController::setAllComplianceMarginSlope( traj_compliance_margin, traj_compliance_slope );
    ROS_INFO("7");
  // -----------------------------------------------------------------------------------------------
  // Log position and velocity error for each joint to file
  std::ofstream error_log_file;
  std::string error_log_string;
  if( USE_ERROR_OUTPUT_LOG )
  {
    std::string error_log_path;
    getUniqueErrorLogPath(error_log_path);

    // Save results to file
    error_log_file.open(error_log_path.c_str());

    // Output list of joint names to first line of erronnr log
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if(!j) // no comma before first item
        error_log_file << joint_names_[j];
      else
        error_log_file << "," << joint_names_[j];
    }
    error_log_file << "\n";
  }
    ROS_INFO("8");
  // Wait until we've reached the trajectory start time
  ROS_INFO("Trajectory start requested at %.3lf, waiting...", traj_msg.header.stamp.toSec());
  ros::Time::sleepUntil(traj_msg.header.stamp);

  ros::Time end_time = traj_msg.header.stamp + ros::Duration(trajectory_duration);
  std::vector<ros::Time> seg_end_times(num_points, ros::Time(0.0));

  for (int i = 0; i < num_points; ++i)
  {
    seg_end_times[i] = ros::Time(trajectory[i].start_time + durations[i]);
  }

  ROS_INFO("Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf", time.toSec(), end_time.toSec(), trajectory_duration);

  trajectory_ = trajectory;
  ros::Time traj_start_time = ros::Time::now();
  ros::Rate rate(update_rate_);

  //------------------------------------------------------------------------------------------------
  // The main loop - sends motor commands once per for loop
  for (int traj_seg = 0; traj_seg < num_points; ++traj_seg)
  {
    ROS_DEBUG("Processing segment %d -------------------------------------------------", traj_seg);

    // first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
    if (durations[traj_seg] == 0.0)
    {
      ROS_DEBUG("Skipping segment %d because duration is 0", traj_seg);
      continue;
    }

    // List of every port, and that port's corresponding commands for every motor
    std::map<std::string, std::vector<std::vector<int> > > multi_port_commands;

    // -----------------------------------------------------------------------------------------
    // Combine all the commands for every motor of every joint that is on the same port into one
    // "multi_port_commands"

    // Loop through every port in this multi joint controller
    for ( std::map<std::string, std::vector<std::string> >::const_iterator port_it =
            port_to_joints_.begin(); port_it != port_to_joints_.end(); ++port_it)
    {
      ROS_DEBUG_STREAM("Processing Port " << port_it->first );

      // List of all commands for a particular port
      std::vector<std::vector<int> > port_motor_commands;

      // Loop through every joint on the port
      for ( std::vector<std::string>::const_iterator joint_it = port_it->second.begin();
            joint_it != port_it->second.end(); ++joint_it)
      {
        // Cache joint data
        int joint_idx = joint_to_idx_[*joint_it];

        // Get start position of this joint
        double start_position;
        if (traj_seg != 0)
        {
          start_position = trajectory[traj_seg-1].positions[joint_idx];
        }
        else
        {
          start_position = joint_states_[*joint_it]->position;
        }

        // Calculate desired values
        double desired_position = trajectory[traj_seg].positions[joint_idx];
        double desired_velocity = std::max<double>(min_velocity_,
                                                   std::abs(desired_position - start_position) /
                                                   durations[traj_seg]);

        ROS_DEBUG("\tstart_position: %f, duration: %f", start_position, durations[traj_seg]);
        ROS_DEBUG("\tport: %s, joint: %s, dpos: %f, dvel: %f", port_it->first.c_str(),
                  joint_it->c_str(), desired_position, desired_velocity);

        // Check that desired_veclocity is not too high, e.g. the position difference not too large
        if( desired_velocity > joint_to_controller_[*joint_it]->getMaxVelocity() )
        {
          traj_result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          error_msg = "Invalid joint trajectory: max velocity exceeded for joint " + *joint_it +
            " with a velocity of " + boost::lexical_cast<std::string>(desired_velocity) +
            " when the max velocity is set to " +
            boost::lexical_cast<std::string>(joint_to_controller_[*joint_it]->getMaxVelocity()) +
            ". On trajectory step " + boost::lexical_cast<std::string>(traj_seg);
          ROS_ERROR("%s", error_msg.c_str());
          if (is_action)
          {
            action_server_->setAborted(traj_result, error_msg);
          }
          return;
        }

        // Generate raw motor commands
        std::vector<std::vector<int> > joint_motor_commands =
          joint_to_controller_[*joint_it]->getRawMotorCommands(desired_position, desired_velocity);

        // Copy raw motor commands to port vector
        for (size_t i = 0; i < joint_motor_commands.size(); ++i)
        {
          port_motor_commands.push_back(joint_motor_commands[i]);
        }

        // Copy port vector to multi port vector
        multi_port_commands[port_it->first] = port_motor_commands;
      }
    }

    // Loop through every port and send it their raw commands
    for ( std::map<std::string, std::vector<std::vector<int> > >::const_iterator
            multi_port_commands_it = multi_port_commands.begin();
          multi_port_commands_it != multi_port_commands.end(); ++multi_port_commands_it)
    {
      port_to_io_[multi_port_commands_it->first]->setMultiPositionVelocity(multi_port_commands_it->second);
    }

    // Now wait for the next segment to be ready to go
    while (time < seg_end_times[traj_seg])
    {
      // check if new trajectory was received, if so abort old one by setting the desired state to current state
      if (is_action && action_server_->isPreemptRequested())
      {
        traj_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        error_msg = "New trajectory received. Aborting old trajectory.";

        std::map<std::string, std::vector<std::vector<int> > > multi_port_commands;

        std::map<std::string, std::vector<std::string> >::const_iterator port_it;
        std::vector<std::string>::const_iterator joint_it;

        for (port_it = port_to_joints_.begin(); port_it != port_to_joints_.end(); ++port_it)
        {
          std::vector<std::vector<int> > port_motor_commands;

          for (joint_it = port_it->second.begin(); joint_it != port_it->second.end(); ++joint_it)
          {
            std::string joint = *joint_it;

            double desired_position = joint_states_[joint]->position;
            double desired_velocity = joint_states_[joint]->velocity;

            std::vector<std::vector<int> > joint_motor_commands = joint_to_controller_[joint]->getRawMotorCommands(desired_position, desired_velocity);
            for (size_t i = 0; i < joint_motor_commands.size(); ++i)
            {
              port_motor_commands.push_back(joint_motor_commands[i]);
            }

            multi_port_commands[port_it->first] = port_motor_commands;
          }
        }

        std::map<std::string, std::vector<std::vector<int> > >::const_iterator multi_port_commands_it;
        for (multi_port_commands_it = multi_port_commands.begin(); multi_port_commands_it != multi_port_commands.end(); ++multi_port_commands_it)
        {
          port_to_io_[multi_port_commands_it->first]->setMultiPositionVelocity(multi_port_commands_it->second);
        }

        action_server_->setPreempted(traj_result, error_msg);
        ROS_WARN("%s", error_msg.c_str());
        return;
      }

      rate.sleep();
      time = ros::Time::now();
    }

    // -----------------------------------------------------------------------------------------
    // Verifies trajectory constraints
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if (trajectory_constraints_[j] > 0.0 && feedback_msg_.error.positions[j] > trajectory_constraints_[j])
      {
        traj_result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
        error_msg = "Unsatisfied position constraint for " + joint_names_[j] +
          " trajectory point " + boost::lexical_cast<std::string>(traj_seg) +
          ", " + boost::lexical_cast<std::string>(feedback_msg_.error.positions[j]) +
          " is larger than " + boost::lexical_cast<std::string>(trajectory_constraints_[j]);
        ROS_ERROR("%s", error_msg.c_str());
        if (is_action)
        {
          action_server_->setAborted(traj_result, error_msg);
        }
        return;
      }

      // Save to file
      if( USE_ERROR_OUTPUT_LOG )
      {
        if(!j) // no comma before first item
          error_log_string = boost::lexical_cast<std::string>(feedback_msg_.error.positions[j]);
        else
          error_log_string += "," + boost::lexical_cast<std::string>(feedback_msg_.error.positions[j]);
      }

    }

    // End line - the entire set of position error for each trajectory point is saved per line
    if( USE_ERROR_OUTPUT_LOG )
    {
      error_log_file << error_log_string << "\n";
    }

  } // end of the main loop

  // let motors roll for specified amount of time
  ros::Duration(goal_time_constraint_).sleep();

  // Check if all motors are within their goal constraints
  bool aborted_at_end = false;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (goal_constraints_[i] > 0 && std::abs(feedback_msg_.error.positions[i]) > goal_constraints_[i])
    {
      traj_result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      error_msg = "Aborting at end because " + joint_names_[i] +
        " joint wound up outside the goal constraints. The position error " +
        boost::lexical_cast<std::string>(fabs(feedback_msg_.error.positions[i])) +
        " is larger than the goal constraints " + boost::lexical_cast<std::string>(goal_constraints_[i]);
      ROS_ERROR("%s", error_msg.c_str());
      if (is_action)
      {
        action_server_->setAborted(traj_result, error_msg);
      }
      aborted_at_end = true;
    }

    // Save final results to file
    if( USE_ERROR_OUTPUT_LOG )
    {
      if(!i) // no comma before first item
        error_log_string = boost::lexical_cast<std::string>(feedback_msg_.error.positions[i]);
      else
        error_log_string += "," + boost::lexical_cast<std::string>(feedback_msg_.error.positions[i]);
    }

  }
  // End line - the entire set of position error for each trajectory point is saved per line
  if( USE_ERROR_OUTPUT_LOG )
  {
    error_log_file << error_log_string << "\n";
  }

  if( aborted_at_end )
    return;

  // -----------------------------------------------------------------------------------------------
  // Finish up

  // Close log file
  if( USE_ERROR_OUTPUT_LOG )
    error_log_file.close();

  // Set result
  traj_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  error_msg = "Trajectory execution successfully completed";
  ROS_INFO("%s", error_msg.c_str());
  action_server_->setSucceeded(traj_result, error_msg);
}


void JointTrajectoryActionController::getUniqueErrorLogPath(std::string &error_log_path)
{
  // Get the location of the dynamixel package within ros
  const std::string package_path = ros::package::getPath("dynamixel_hardware_interface");

  std::string error_log_base_path = package_path + "/data/trajectory_error_";

  // Loop until we find a number-appended file name that does not exist
  unsigned int file_id = 0;
  do
  {
    error_log_path = error_log_base_path + boost::lexical_cast<std::string>(file_id) + ".csv";
    ++file_id;
  } while( boost::filesystem::is_regular_file( error_log_path ) );

  ROS_INFO_STREAM("Error logging to " << error_log_path);
  //error_log_path = "/home/dave/ros/clam/src/dynamixel_hardware_interface/data";
}

}
