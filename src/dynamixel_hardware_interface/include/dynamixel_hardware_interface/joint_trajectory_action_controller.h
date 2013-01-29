#ifndef DYNAMIXEL_HARDWARE_INTERFACE_JOINT_TRAJECTORY_ACTION_CONTROLLER_H
#define DYNAMIXEL_HARDWARE_INTERFACE_JOINT_TRAJECTORY_ACTION_CONTROLLER_H

#include <vector>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/multi_joint_controller.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace controller
{

struct Segment
{
  double start_time;
  double duration;
  std::vector<double> positions;
  std::vector<double> velocities;
};

class JointTrajectoryActionController : public MultiJointController
{
public:
  JointTrajectoryActionController();
  virtual ~JointTrajectoryActionController();

  bool initialize(std::string name, std::vector<boost::shared_ptr<controller::SingleJointController> > deps);

  void start();
  void stop();

  void processCommand(const trajectory_msgs::JointTrajectoryConstPtr& msg);
  void processFollowTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  void updateState();
  void processTrajectory(const trajectory_msgs::JointTrajectory& traj, bool is_action);

private:
  void getUniqueErrorLogPath(std::string &log_path);

  int update_rate_;
  int state_update_rate_;
  std::vector<Segment> trajectory_;

  double goal_time_constraint_;
  double stopped_velocity_tolerance_;
  double min_velocity_;
  std::vector<double> goal_constraints_;
  std::vector<double> trajectory_constraints_;

  control_msgs::FollowJointTrajectoryFeedback feedback_msg_;

  ros::Subscriber command_sub_;
  ros::Publisher state_pub_;

  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  boost::scoped_ptr<FJTAS> action_server_;

  boost::thread* feedback_thread_;
  boost::mutex terminate_mutex_;
  bool terminate_;

};

}

#endif  // DYNAMIXEL_HARDWARE_INTERFACE_JOINT_TRAJECTORY_ACTION_CONTROLLER_H
