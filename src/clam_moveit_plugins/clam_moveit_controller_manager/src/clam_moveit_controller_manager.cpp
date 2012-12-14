/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_list_macros.h>

//#include <dynamixel_hardware_interface/SwitchController.h>
#include <dynamixel_hardware_interface/StartController.h>
#include <dynamixel_hardware_interface/StopController.h>
#include <dynamixel_hardware_interface/ListControllers.h>
#include <algorithm>
#include <map>

namespace clam_moveit_controller_manager
{

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// ClamFollowJointTrajectoryControllerHandle
// TODO: convert this
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
class ClamFollowJointTrajectoryControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:

  ClamFollowJointTrajectoryControllerHandle(const std::string &name, const std::string &ns = "follow_joint_trajectory") :
    moveit_controller_manager::MoveItControllerHandle(name), namespace_(ns), done_(true)
  {
    follow_joint_trajectory_action_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(name_ + "/" + namespace_, true));
    unsigned int attempts = 0;
    while (ros::ok() && !follow_joint_trajectory_action_client_->waitForServer(ros::Duration(5.0)) && ++attempts < 3)
      ROS_INFO_STREAM("Waiting for the follow joint trajectory action for controller " << name_ + "/" + namespace_ << " to come up");

    if (!follow_joint_trajectory_action_client_->isServerConnected())
    {
      ROS_ERROR_STREAM("Action client not connected for joint trajectory controller " << name_ + "/" + namespace_);
      follow_joint_trajectory_action_client_.reset();
    }

    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  bool isConnected(void) const
  {
    return follow_joint_trajectory_action_client_;
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    if (!follow_joint_trajectory_action_client_)
      return false;
    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR("The Clam FollowJointTrajectory controller cannot execute multi-dof trajectories.");
      return false;
    }
    if (done_)
      ROS_INFO_STREAM("Sending trajectory to FollowJointTrajectory action for controller " << name_);
    else
      ROS_INFO_STREAM("Sending continuation for the currently executed trajectory to FollowJointTrajectory action for controller " << name_);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;
    follow_joint_trajectory_action_client_->sendGoal(goal,
                                                     boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
                                                     boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
                                                     boost::bind(&ClamFollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  virtual bool cancelExecution(void)
  {
    if (!follow_joint_trajectory_action_client_)
      return false;
    if (!done_)
    {
      ROS_INFO_STREAM("Cancelling execution of trajectory on controller " << name_);
      follow_joint_trajectory_action_client_->cancelGoal();
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }

  virtual bool waitForExecution(const ros::Duration &timeout = ros::Duration(0))
  {
    if (follow_joint_trajectory_action_client_ && !done_)
      return follow_joint_trajectory_action_client_->waitForResult(timeout);
    return true;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus(void)
  {
    return last_exec_;
  }

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    ROS_INFO_STREAM("Controller " << name_ << " is done with state " << state.toString() << ": " << state.getText());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else
      if (state == actionlib::SimpleClientGoalState::ABORTED)
        last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      else
        if (state == actionlib::SimpleClientGoalState::PREEMPTED)
          last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        else
          last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  void controllerActiveCallback(void)
  {
    ROS_INFO_STREAM("Controller " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
  {
  }

protected:

  moveit_controller_manager::ExecutionStatus last_exec_;
  std::string namespace_;
  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > follow_joint_trajectory_action_client_;
  bool done_;
};


// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// ClamMoveItControllerManager
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
class ClamMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  ClamMoveItControllerManager(void) : node_handle_("~")
  {
    node_handle_.param("controller_manager_name", controller_manager_name_, std::string("clam_controller_manager"));
    node_handle_.param("use_controller_manager", use_controller_manager_, true);

    // Read controller_list from parameter server and converto to type ControllerInformation
    XmlRpc::XmlRpcValue controller_list;
    if (node_handle_.hasParam("controller_list"))
    {
      node_handle_.getParam("controller_list", controller_list);
      if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_WARN("Controller list should be specified as an array");
      else
        for (int i = 0 ; i < controller_list.size() ; ++i)
        {
          if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
            ROS_WARN("Name and joints must be specifed for each controller");
          else
          {
            try
            {
              ControllerInformation ci;
              std::string name = std::string(controller_list[i]["name"]);
              if (controller_list[i].hasMember("default"))
              {
                try
                {
                  if (controller_list[i]["default"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                  {
                    ci.default_ = controller_list[i]["default"];
                  }
                  else
                  {
                    std::string def = std::string(controller_list[i]["default"]);
                    std::transform(def.begin(), def.end(), def.begin(), ::tolower);
                    if (def == "true" || def == "yes")
                      ci.default_ = true;
                  }
                }
                catch (...)
                {
                }
              }
              if (controller_list[i].hasMember("ns"))
                ci.ns_ = std::string(controller_list[i]["ns"]);

              if (controller_list[i]["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray)
              {

                int nj = controller_list[i]["joints"].size();

                for (int j = 0 ; j < nj ; ++j)
                {
                  ci.joints_.push_back(std::string(controller_list[i]["joints"][j]));
                }
              }
              else
                ROS_WARN_STREAM("The list of joints for controller " << name << " is not specified as an array");
              if (!ci.joints_.empty())
              {
                possibly_unloaded_controllers_[name] = ci;
              }
            }
            catch (...)
            {
              ROS_ERROR("Unable to parse controller information");
            }
          }
        }
    }
    else
    {
      if (use_controller_manager_)
        ROS_INFO_STREAM("No controller list specified. Using list obtained from the " << controller_manager_name_);
      else
        ROS_ERROR_STREAM("Not using a controller manager and no controllers specified. There are no known controllers.");
    }

    if (use_controller_manager_)
    {
      static const unsigned int max_attempts = 5;
      unsigned int attempts = 0;
      while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/list_controllers", ros::Duration(5.0)) && ++attempts < max_attempts)
        ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/list_controllers" << " to come up");

      //if (attempts < max_attempts)
      //  while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/switch_controller", ros::Duration(5.0)) && ++attempts < max_attempts)
      //    ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/switch_controller" << " to come up");

      if (attempts < max_attempts)
        while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/start_controller", ros::Duration(5.0))  && ++attempts < max_attempts)
          ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/start_controller" << " to come up");

      if (attempts < max_attempts)
        while (ros::ok() && !ros::service::waitForService(controller_manager_name_ + "/stop_controller", ros::Duration(5.0))  && ++attempts < max_attempts)
          ROS_INFO_STREAM("Waiting for service " << controller_manager_name_ + "/stop_controller" << " to come up");

      if (attempts < max_attempts)
      {
        lister_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::ListControllers>(controller_manager_name_ + "/list_controllers", true);
        //switcher_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::SwitchController>(controller_manager_name_ + "/switch_controller", true);
        loader_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::StartController>(controller_manager_name_ + "/start_controller", true);
        unloader_service_ = root_node_handle_.serviceClient<dynamixel_hardware_interface::StopController>(controller_manager_name_ + "/stop_controller", true);
      }
      else
        ROS_ERROR("Not using the Clam controller manager");
    }
  }

  virtual ~ClamMoveItControllerManager(void)
  {
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr>::const_iterator it = handle_cache_.find(name);
    if (it != handle_cache_.end())
      return it->second;

    moveit_controller_manager::MoveItControllerHandlePtr new_handle;
    if (possibly_unloaded_controllers_.find(name) != possibly_unloaded_controllers_.end())
    {
      const std::string &ns = possibly_unloaded_controllers_.at(name).ns_;
      if (!ns.empty())
        new_handle.reset(new ClamFollowJointTrajectoryControllerHandle(name, ns));
    }
    if (!new_handle)
      new_handle.reset(new ClamFollowJointTrajectoryControllerHandle(name));
    if (new_handle && static_cast<ClamFollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
      handle_cache_[name] = new_handle;
    return new_handle;
  }

  virtual void getControllersList(std::vector<std::string> &names)
  {
    const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
    std::set<std::string> names_set;
    names_set.insert(res.controllers.begin(), res.controllers.end());

    for (std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.begin() ; it != possibly_unloaded_controllers_.end() ; ++it)
      names_set.insert(it->first);

    names.clear();
    names.insert(names.end(), names_set.begin(), names_set.end());
  }

  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    names.clear();
    const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
    for (std::size_t i = 0; i < res.controllers.size(); ++i)
      if (res.state[i] == "running")
        names.push_back(res.controllers[i]);
  }

  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
    names = res.controllers;
  }

  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.find(name);
    if (it != possibly_unloaded_controllers_.end())
      joints = it->second.joints_;
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
  }

  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    const dynamixel_hardware_interface::ListControllers::Response &res = getListControllerServiceResponse();
    for (std::size_t i = 0; i < res.controllers.size(); ++i)
    {
      if (res.controllers[i] == name)
      {
        state.loaded_ = true;
        if (res.state[i] == "running")
          state.active_ = true;
        break;
      }
    }
    std::map<std::string, ControllerInformation>::const_iterator it = possibly_unloaded_controllers_.find(name);
    if (it != possibly_unloaded_controllers_.end())
      if (it->second.default_)
        state.default_ = true;
    return state;
  }

  virtual bool loadController(const std::string &name)
  {
    ROS_INFO("Recieved load controller request");

    if (!use_controller_manager_)
    {
      ROS_WARN_STREAM("Cannot load controller without using the controller manager");
      return false;
    }
    last_lister_response_ = ros::Time();
    handle_cache_.erase(name);
    dynamixel_hardware_interface::StartController::Request req;
    dynamixel_hardware_interface::StartController::Response res;
    req.name = name;
    req.port = ""; // TODO - add ability in dynamixel to know the port
    if (!loader_service_.call(req, res))
    {
      ROS_WARN_STREAM("Something went wrong with loader service");
      return false;
    }
    if (!res.success)
      ROS_WARN_STREAM("Loading controller " << name << " failed");
    return res.success;
  }

  virtual bool unloadController(const std::string &name)
  {
    ROS_INFO("Recieved unload controller request");

    if (!use_controller_manager_)
    {
      ROS_WARN_STREAM("Cannot unload controller without using the controller manager");
      return false;
    }
    last_lister_response_ = ros::Time();
    handle_cache_.erase(name);
    dynamixel_hardware_interface::StopController::Request req;
    dynamixel_hardware_interface::StopController::Response res;
    req.name = name;
    if (!unloader_service_.call(req, res))
    {
      ROS_WARN_STREAM("Something went wrong with unloader service");
      return false;
    }
    if (!res.success)
      ROS_WARN_STREAM("Unloading controller " << name << " failed");
    return res.success;
  }

  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate)
  {
    if (!use_controller_manager_)
    {
      ROS_WARN_STREAM("Cannot switch controllers without using the controller manager");
      return false;
    }
    last_lister_response_ = ros::Time();
    /*
      dynamixel_hardware_interface::SwitchController::Request req;
      dynamixel_hardware_interface::SwitchController::Response res;

      req.strictness = dynamixel_hardware_interface::SwitchController::Request::BEST_EFFORT;
      req.start_controllers = activate;
      req.stop_controllers = deactivate;
      if (!switcher_service_.call(req, res))
      {
      ROS_WARN_STREAM("Something went wrong with switcher service");
      return false;
      }
      if (!res.success)
      ROS_WARN_STREAM("Switcher service failed");
      return res.success;
    */

    ROS_WARN_STREAM("Something went wrong with switcher service");
    return false;
  }


protected:


  const dynamixel_hardware_interface::ListControllers::Response &getListControllerServiceResponse(void)
  {
    if (use_controller_manager_)
    {
      static const ros::Duration max_cache_age(10.0);
      if ((ros::Time::now() - last_lister_response_) > max_cache_age)
      {
        dynamixel_hardware_interface::ListControllers::Request req;
        if (!lister_service_.call(req, cached_lister_response_))
          ROS_WARN_STREAM("Something went wrong with lister service");
        last_lister_response_ = ros::Time::now();
      }
    }
    return cached_lister_response_;
  }


  ros::NodeHandle node_handle_;
  ros::NodeHandle root_node_handle_;

  std::string controller_manager_name_;
  bool use_controller_manager_;
  ros::ServiceClient loader_service_;
  ros::ServiceClient unloader_service_;
  ros::ServiceClient switcher_service_;
  ros::ServiceClient lister_service_;

  ros::Time last_lister_response_;
  dynamixel_hardware_interface::ListControllers::Response cached_lister_response_;

  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> handle_cache_;

  struct ControllerInformation
  {
    ControllerInformation(void) : default_(false)
    {
    }

    bool default_;
    std::string ns_;
    std::vector<std::string> joints_;
  };
  std::map<std::string, ControllerInformation> possibly_unloaded_controllers_;
};


}

PLUGINLIB_DECLARE_CLASS(clam_moveit_controller_manager, ClamMoveItControllerManager,
                        clam_moveit_controller_manager::ClamMoveItControllerManager,
                        moveit_controller_manager::MoveItControllerManager);
