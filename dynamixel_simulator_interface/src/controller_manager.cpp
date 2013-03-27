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

#include <string>
#include <map>
#include <XmlRpcValue.h>
#include <boost/foreach.hpp>

#include <dynamixel_simulator_interface/controller_manager.h>
//#include <dynamixel_simulator_interface/serial_proxy.h>
#include <dynamixel_simulator_interface/single_joint_controller.h>
#include <dynamixel_simulator_interface/multi_joint_controller.h>
#include <dynamixel_hardware_interface/JointState.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
//#include <diagnostic_updater/DiagnosticStatusWrapper.h>
//#include <diagnostic_msgs/DiagnosticArray.h>

#include <dynamixel_hardware_interface/LoadController.h>
#include <dynamixel_hardware_interface/UnloadController.h>
#include <dynamixel_hardware_interface/RestartController.h>
#include <dynamixel_hardware_interface/ListControllers.h>

namespace dynamixel_controller_manager
{

ControllerManager::ControllerManager() : nh_(ros::NodeHandle()), private_nh_(ros::NodeHandle("~"))
{
  private_nh_.param<double>("diagnostics_rate", diagnostics_rate_, 1.0);

  if (!private_nh_.getParam("namespace", manager_namespace_))
  {
    ROS_ERROR("dynamixel_controller_manager requires namespace paramater to be set");
  }

  XmlRpc::XmlRpcValue serial_ports;

  if (!private_nh_.getParam("serial_ports", serial_ports))
  {
    ROS_ERROR("dynamixel_controller_manager requires serial_ports parameter to be set");
  }

  if (serial_ports.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("dynamixel_controller_manager serial_ports has to be a map, passed type is %d", serial_ports.getType());
  }

  /*
    std::string port_namespace;
    XmlRpc::XmlRpcValue::iterator it;

    for (it = serial_ports.begin(); it != serial_ports.end(); ++it)
    {
    port_namespace = (*it).first;
    std::string prefix = "serial_ports/" + port_namespace + "/";

    std::string port_name;
    private_nh_.param<std::string>(prefix + "port_name", port_name, "/dev/ttyUSB0");

    int baud_rate_int;
    private_nh_.param<int>(prefix + "baud_rate", baud_rate_int, 1000000);
    std::string baud_rate = boost::lexical_cast<std::string>(baud_rate_int);

    int min_motor_id;
    private_nh_.param<int>(prefix + "min_motor_id", min_motor_id, 1);

    int max_motor_id;
    private_nh_.param<int>(prefix + "max_motor_id", max_motor_id, 25);

    int update_rate;
    private_nh_.param<int>(prefix + "update_rate", update_rate, 10);

    prefix += "diagnostics/";

    int error_level_temp;
    private_nh_.param<int>(prefix + "error_level_temp", error_level_temp, 65);

    int warn_level_temp;
    private_nh_.param<int>(prefix + "warn_level_temp", warn_level_temp, 60);

    dynamixel_simulator_interface::SerialProxy* serial_proxy =
    new dynamixel_simulator_interface::SerialProxy(port_name,
    port_namespace,
    baud_rate,
    min_motor_id,
    max_motor_id,
    update_rate,
    diagnostics_rate_,
    error_level_temp,
    warn_level_temp);
    if (!serial_proxy->connect())
    {
    delete serial_proxy;
    continue;
    }

    serial_proxies_[port_namespace] = serial_proxy;
    }

    if (serial_proxies_.empty())
    {
    ROS_FATAL("No motors found on any configured serial port, aborting");
    exit(1);
    }
  */
  sjc_loader_.reset(new pluginlib::ClassLoader<controller::SingleJointController>("dynamixel_simulator_interface", "controller::SingleJointController"));
  mjc_loader_.reset(new pluginlib::ClassLoader<controller::MultiJointController>("dynamixel_simulator_interface", "controller::MultiJointController"));

  //  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);

  load_controller_server_ = nh_.advertiseService(manager_namespace_ + "/load_controller",
                                                  &ControllerManager::startControllerSrv, this);

  unload_controller_server_ = nh_.advertiseService(manager_namespace_ + "/unload_controller",
                                                 &ControllerManager::stopControllerSrv, this);

  reload_controller_server_ = nh_.advertiseService(manager_namespace_ + "/reload_controller",
                                                    &ControllerManager::restartControllerSrv, this);

  list_controllers_server_ = nh_.advertiseService(manager_namespace_ + "/list_controllers",
                                                  &ControllerManager::listControllersSrv, this);

}

ControllerManager::~ControllerManager()
{
  /*
    if (diagnostics_thread_)
    {
    {
    boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
    terminate_diagnostics_ = true;
    }
    diagnostics_thread_->join();
    delete diagnostics_thread_;
    }
  */

  /*
    std::map<std::string, dynamixel_simulator_interface::SerialProxy*>::iterator sp_it;

    for (sp_it = serial_proxies_.begin(); sp_it != serial_proxies_.end(); ++sp_it)
    {
    delete sp_it->second;
    }

    std::map<std::string, boost::shared_ptr<controller::SingleJointController> >::iterator c_it;

    for (c_it = sj_controllers_.begin(); c_it != sj_controllers_.end(); ++c_it)
    {
    delete c_it->second;
    }
  */
}

bool ControllerManager::startController(std::string name, std::string port)
{
  boost::mutex::scoped_lock c_guard(controllers_lock_);
  ROS_DEBUG("Will start controller '%s'", name.c_str());

  std::string type;
  if (!nh_.getParam(name + "/type", type))
  {
    ROS_ERROR("Type not specified for %s controller", name.c_str());
    return false;
  }

  // assume we are loading a single joint controller, then look for its
  // name in declared multi-joint controllers, if found we are loading
  // a multi-joint controller
  bool single_joint_controller = true;
  std::vector<std::string> mj_classes = mjc_loader_->getDeclaredClasses();
  for (size_t i = 0; i < mj_classes.size(); ++i)
  {
    if (mj_classes[i] == type)
    {
      single_joint_controller = false;
      break;
    }
  }

  if (single_joint_controller)
  {
    ROS_DEBUG("Loading single-joint controller");

    /*
      if (port.empty())
      {
      ROS_ERROR("Port name is not specified for controller %s", name.c_str());
      return false;
      }

      if (serial_proxies_.find(port) == serial_proxies_.end())
      {
      ROS_ERROR("Serial port %s is not managed by %s controller manager", port.c_str(), manager_namespace_.c_str());
      return false;
      }
    */

    if (sj_controllers_.find(name) != sj_controllers_.end())
    {
      ROS_ERROR("Controller %s is already started", name.c_str());
      return false;
    }

    boost::shared_ptr<controller::SingleJointController> sjc; // = NULL;
    ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());

    try
    {
      //      c = sjc_loader_->createClassInstance(type, true);
      sjc = sjc_loader_->createInstance(type);
    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    // checks if controller was constructed
    if (sjc == NULL)
    {
      if (type == "")
      {
        ROS_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server?", name.c_str());
      }
      else
      {
        ROS_ERROR("Could not load controller '%s' because controller type '%s' does not exist", name.c_str(), type.c_str());
      }

      return false;
    }

    // Initializes the controller
    ROS_DEBUG("Initializing controller '%s'", name.c_str());
    bool initialized = false;

    try
    {
      dynamixel_simulator_interface::DynamixelIO* dxl_io;
      ROS_DEBUG_STREAM_NAMED("controller_manager","Initializing controller " << name);
      initialized = sjc->initialize(name, port, dxl_io);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while initializing controller %s.\n%s", name.c_str(), e.what());
      initialized = false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while initializing controller %s", name.c_str());
      initialized = false;
    }

    if (!initialized)
    {
      //      delete sjc;
      ROS_ERROR("Initializing controller '%s' failed", name.c_str());
      return false;
    }
    sjc->start();
    sj_controllers_[name] = sjc;
    ROS_DEBUG("Initialized controller '%s' successful", name.c_str());
  }
  else
  {
    ROS_DEBUG("Loading multi-joint controller");

    std::vector<std::string> dependencies;
    XmlRpc::XmlRpcValue raw;
    if (!nh_.getParam(name + "/dependencies", raw))
    {
      ROS_ERROR("Dependencies are not specified for multi-joint controller %s", name.c_str());
      return false;
    }

    if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Dependencies parameter of controller %s is not a list", name.c_str());
      return false;
    }

    for (int i = 0; i < raw.size(); ++i)
    {
      if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("All dependencies of controller %s should be strings", name.c_str());
        return false;
      }

      dependencies.push_back(static_cast<std::string>(raw[i]));
    }

    if (mj_controllers_.find(name) != mj_controllers_.end() ||
        mj_waiting_controllers_.find(name) != mj_waiting_controllers_.end())
    {
      ROS_ERROR("Multi-joint controller %s is already started", name.c_str());
      return false;
    }

    std::pair<std::string, std::vector<std::string> > mjc_spec(name, dependencies);
    mj_waiting_controllers_.insert(name);
    waiting_mjcs_.insert(mjc_spec);
  }

  checkDeps();
  return true;
}

bool ControllerManager::stopController(std::string name)
{
  boost::mutex::scoped_lock c_guard(controllers_lock_);
  ROS_DEBUG("Will stop controller '%s'", name.c_str());

  std::map<std::string, boost::shared_ptr<controller::SingleJointController> >::iterator sit;
  std::map<std::string, boost::shared_ptr<controller::MultiJointController> >::iterator mit;

  sit = sj_controllers_.find(name);
  mit = mj_controllers_.find(name);

  if (sit == sj_controllers_.end())
  {
    ROS_DEBUG("%s not found in single_joint_controller map", name.c_str());

    if (mit == mj_controllers_.end())
    {
      ROS_ERROR("Unable to stop %s, controller is not running", name.c_str());
      return false;
    }
    else
    {
      boost::shared_ptr<controller::MultiJointController>  mjc = mit->second;
      ROS_DEBUG("stopping multi-joint controller %s", name.c_str());
      mjc->stop();
      mj_controllers_.erase(mit);
      //delete c;
    }
  }
  // trying to stop a single-joint controller
  else
  {
    // first check if any of the loaded multi-joint controllers require this
    // single-joint controller (i.e. have it in their dependencies list)
    std::map<std::string, boost::shared_ptr<controller::MultiJointController> >::iterator it;

    for (it = mj_controllers_.begin(); it != mj_controllers_.end(); ++it)
    {
      // get dependencies
      std::vector<boost::shared_ptr<controller::SingleJointController> > deps = it->second->getDependencies();

      // go through the list and compare each dep name to passed in controller name
      for (size_t i = 0; i < deps.size(); ++i)
      {
        std::string dep_name = deps[i]->getName();

        // found a match means this multi-joint controller depends on the single-joint
        // controller we are trying to stop, report failure
        if (dep_name == name)
        {
          ROS_ERROR("Can not stop single-joint controller %s, multi-joint controller %s still depends on it", name.c_str(), it->first.c_str());
          return false;
        }
      }
    }

    boost::shared_ptr<controller::SingleJointController> sjc = sit->second;
    ROS_DEBUG("stopping single-joint controller %s", name.c_str());
    sjc->stop();
    sj_controllers_.erase(sit);
    //delete c;
  }

  return true;
}

bool ControllerManager::restartController(std::string name)
{
  ROS_DEBUG("Will restart controller '%s'", name.c_str());

  std::string port;

  {
    boost::mutex::scoped_lock c_guard(controllers_lock_);
    std::map<std::string, boost::shared_ptr<controller::SingleJointController> >::const_iterator sit;
    sit = sj_controllers_.find(name);

    // if not found in both single and multi-joint maps, it is not loaded and running
    if (sit == sj_controllers_.end())
    {
      if (mj_controllers_.find(name) == mj_controllers_.end())
      {
        ROS_ERROR("Controller %s is not running", name.c_str());
        return false;
      }
    }
    else
    {
      port = sit->second->getPortNamespace();
    }
  }

  if (stopController(name) && startController(name, port)) { return true; }
  return false;
}



void ControllerManager::checkDeps()
{
  std::set<std::string> loaded_controllers;
  std::map<std::string, boost::shared_ptr<controller::SingleJointController> >::iterator c_it;

  for (c_it = sj_controllers_.begin(); c_it != sj_controllers_.end(); ++c_it)
  {
    loaded_controllers.insert(c_it->first);
  }

  std::set<std::pair<std::string, std::vector<std::string> > >::iterator it;

  for (it = waiting_mjcs_.begin(); it != waiting_mjcs_.end(); )
  {
    std::string name = it->first;
    std::set<std::string> deps(it->second.begin(), it->second.end());

    if (std::includes(loaded_controllers.begin(), loaded_controllers.end(), deps.begin(), deps.end()))
    {
      ROS_DEBUG("All dependencies are loaded for multi-joint controller %s", name.c_str());

      std::vector<boost::shared_ptr<controller::SingleJointController> > dependencies;
      std::set<std::string>::iterator d_it;

      for (d_it = deps.begin(); d_it != deps.end(); ++d_it)
      {
        dependencies.push_back(sj_controllers_[*d_it]);
      }

      boost::shared_ptr<controller::MultiJointController>  mjc; // = NULL;
      std::string type;

      if (nh_.getParam(name + "/type", type))
      {
        ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());

        try
        {
          //          c = mjc_loader_->createClassInstance(type, true);
          mjc = mjc_loader_->createInstance(type);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          //handle the class failing to load
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }

      // checks if controller was constructed
      if (mjc == NULL)
      {
        if (type == "")
        {
          ROS_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server?", name.c_str());
        }
        else
        {
          ROS_ERROR("Could not load controller '%s' because controller type '%s' does not exist", name.c_str(), type.c_str());
        }

        ++it;
        continue;
      }

      // Initializes the controller
      ROS_DEBUG("Initializing controller '%s'", name.c_str());
      bool initialized = false;

      try
      {
        initialized = mjc->initialize(name, dependencies);
      }
      catch(std::exception &e)
      {
        ROS_ERROR("Exception thrown while initializing controller %s.\n%s", name.c_str(), e.what());
        initialized = false;
      }
      catch(...)
      {
        ROS_ERROR("Exception thrown while initializing controller %s", name.c_str());
        initialized = false;
      }

      if (!initialized)
      {
        ROS_ERROR("Initializing controller '%s' failed", name.c_str());
        ++it;
        continue;
      }

      mjc->start();

      mj_controllers_[name] = mjc;
      mj_waiting_controllers_.erase(name);
      waiting_mjcs_.erase(it++);

      ROS_DEBUG("Initialized controller '%s' successful", name.c_str());
    }
    else
    {
      std::vector<std::string> diff(deps.size());
      std::set_difference(deps.begin(), deps.end(), loaded_controllers.begin(), loaded_controllers.end(), diff.begin());

      std::string still_waiting = "[";

      for (size_t i = 0; i < diff.size(); ++i)
      {
        if (!diff[i].empty()) { still_waiting += diff[i] + ", "; }
      }

      if (diff.empty()) { still_waiting = ""; }
      else { still_waiting = still_waiting.erase(still_waiting.size() - 2) + "]"; }

      ROS_INFO("Multi-joint controller %s is still waiting for %s single-joint controllers", name.c_str(), still_waiting.c_str());

      ++it;
    }
  }
}

bool ControllerManager::startControllerSrv(dynamixel_hardware_interface::LoadController::Request& req,
                                           dynamixel_hardware_interface::LoadController::Response& res)
{
  std::string name = req.name;

  if (name.empty())
  {
    ROS_ERROR("Controller name is not specified");
    res.ok = false;
    return false;
  }

  ROS_DEBUG("Start service called for controller %s ", name.c_str());
  boost::mutex::scoped_lock s_guard(services_lock_);
  ROS_DEBUG("Start service locked");

  if (!startController(name, req.port))
  {
    res.ok = false;
    return false;
  }

  ROS_DEBUG("Controller %s successfully started", name.c_str());
  res.ok = true;
  return true;
}

bool ControllerManager::stopControllerSrv(dynamixel_hardware_interface::UnloadController::Request& req,
                                          dynamixel_hardware_interface::UnloadController::Response& res)
{
  std::string name = req.name;

  if (name.empty())
  {
    ROS_ERROR("Controller name is not specified");
    res.ok = false;
    return false;
  }

  ROS_DEBUG("Stop service called for controller %s ", name.c_str());
  boost::mutex::scoped_lock s_guard(services_lock_);
  ROS_DEBUG("Stop service locked");

  if (!stopController(name))
  {
    res.ok = false;
    return false;
  }

  ROS_DEBUG("Controller %s successfully stopped", name.c_str());
  res.ok = true;
  return true;
}

bool ControllerManager::restartControllerSrv(dynamixel_hardware_interface::RestartController::Request& req,
                                             dynamixel_hardware_interface::RestartController::Response& res)
{
  std::string name = req.name;

  if (name.empty())
  {
    ROS_ERROR("Controller name is not specified");
    res.ok = false;
    return false;
  }

  ROS_DEBUG("Restart service called for controller %s ", name.c_str());
  boost::mutex::scoped_lock s_guard(services_lock_);
  ROS_DEBUG("Restart service locked");

  if (!restartController(name))
  {
    res.ok = false;
    return false;
  }

  res.ok = true;
  return true;
}

bool ControllerManager::listControllersSrv(dynamixel_hardware_interface::ListControllers::Request& req,
                                           dynamixel_hardware_interface::ListControllers::Response& res)
{
  ROS_DEBUG("List service called");

  // Resize the response to the number of combined controllers
  res.controllers.resize( mj_controllers_.size() );
  res.state.resize( mj_controllers_.size() );

  int i = 0;

  /*
  // Insert single joint controllers
  for (std::map<std::string, boost::shared_ptr<controller::SingleJointController> >::const_iterator it =
  sj_controllers_.begin(); it != sj_controllers_.end() ; ++it)
  {
  std::cout << "Controller = " << it->first << std::endl;

  res.controllers[i] = std::string(it->first);
  res.state[i] = std::string("running"); // TODO: make this work

  ++i;
  }
  */

  // Insert multi joint controllers
  for (std::map<std::string, boost::shared_ptr<controller::MultiJointController> >::const_iterator it =
         mj_controllers_.begin(); it != mj_controllers_.end() ; ++it)
  {
    res.controllers[i] = std::string(it->first);
    res.state[i] = std::string("running"); // TODO: make this work
  }

  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  dynamixel_controller_manager::ControllerManager cm;
  ros::spin();
  return 1;
}
