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


#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include <map>
#include <set>
#include <string>

#include <boost/thread.hpp>

//#include <dynamixel_simulator_interface/serial_proxy.h>
#include <dynamixel_simulator_interface/single_joint_controller.h>
#include <dynamixel_simulator_interface/multi_joint_controller.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <dynamixel_hardware_interface/LoadController.h>
#include <dynamixel_hardware_interface/UnloadController.h>
#include <dynamixel_hardware_interface/RestartController.h>
#include <dynamixel_hardware_interface/ListControllers.h>

namespace dynamixel_controller_manager
{

class ControllerManager
{

public:
  ControllerManager();
  virtual ~ControllerManager();

  bool startController(std::string name, std::string port);
  bool stopController(std::string name);
  bool restartController(std::string name);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string manager_namespace_;

  double diagnostics_rate_;
  ros::Publisher diagnostics_pub_;

  ros::ServiceServer load_controller_server_;
  ros::ServiceServer unload_controller_server_;
  ros::ServiceServer reload_controller_server_;
  ros::ServiceServer list_controllers_server_;

  //  std::map<std::string, dynamixel_simulator_interface::SerialProxy*> serial_proxies_;

  boost::thread* diagnostics_thread_;

  boost::mutex terminate_mutex_;
  bool terminate_diagnostics_;

  boost::shared_ptr<pluginlib::ClassLoader<controller::SingleJointController> > sjc_loader_;
  boost::shared_ptr<pluginlib::ClassLoader<controller::MultiJointController> > mjc_loader_;

  std::map<std::string, boost::shared_ptr<controller::SingleJointController> > sj_controllers_;
  std::map<std::string, boost::shared_ptr<controller::MultiJointController> > mj_controllers_;

  std::set<std::string> mj_waiting_controllers_;
  std::set<std::pair<std::string, std::vector<std::string> > > waiting_mjcs_;

  boost::mutex controllers_lock_;
  boost::mutex services_lock_;

  void publishDiagnosticInformation();
  void checkDeps();

  bool startControllerSrv(dynamixel_hardware_interface::LoadController::Request& req,
                          dynamixel_hardware_interface::LoadController::Response& res);

  bool stopControllerSrv(dynamixel_hardware_interface::UnloadController::Request& req,
                         dynamixel_hardware_interface::UnloadController::Response& res);

  bool restartControllerSrv(dynamixel_hardware_interface::RestartController::Request& req,
                            dynamixel_hardware_interface::RestartController::Response& res);

  bool listControllersSrv(dynamixel_hardware_interface::ListControllers::Request& req,
                          dynamixel_hardware_interface::ListControllers::Response& res);

};

}

#endif // CONTROLLER_MANAGER_H
