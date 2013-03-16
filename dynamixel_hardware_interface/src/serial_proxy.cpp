
// Author: Antons Rebguns

#include <stdint.h>
#include <time.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <XmlRpcValue.h>

#include <clam/gearbox/flexiport/flexiport.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/serial_proxy.h>
#include <dynamixel_hardware_interface/MotorState.h>
#include <dynamixel_hardware_interface/MotorStateList.h>

#include <ros/ros.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace dynamixel_hardware_interface
{

SerialProxy::SerialProxy(std::string port_name,
                         std::string port_namespace,
                         std::string baud_rate,
                         int min_motor_id,
                         int max_motor_id,
                         double update_rate,
                         double diagnostics_rate,
                         int error_level_temp,
                         int warn_level_temp)
  :port_name_(port_name),
   port_namespace_(port_namespace),
   baud_rate_(baud_rate),
   min_motor_id_(min_motor_id),
   max_motor_id_(max_motor_id),
   update_rate_(update_rate),
   diagnostics_rate_(diagnostics_rate),
   error_level_temp_(error_level_temp),
   warn_level_temp_(warn_level_temp),
   freq_status_(diagnostic_updater::FrequencyStatusParam(&update_rate_, &update_rate_, 0.1, 25))
{
  current_state_ = MotorStateListPtr(new MotorStateList);

  motor_states_pub_ = nh_.advertise<MotorStateList>("motor_states/" + port_namespace_, 1000);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);
}

SerialProxy::~SerialProxy()
{
  if (feedback_thread_)
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      terminate_feedback_ = true;
    }
    feedback_thread_->join();
    delete feedback_thread_;
  }

  if (diagnostics_thread_)
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      terminate_diagnostics_ = true;
    }
    diagnostics_thread_->join();
    delete diagnostics_thread_;
  }

  delete dxl_io_;
}

bool SerialProxy::connect()
{
  try
  {
    ROS_DEBUG("Constructing serial_proxy with %s at %s baud", port_name_.c_str(), baud_rate_.c_str());
    dxl_io_ = new DynamixelIO(port_name_, baud_rate_);
    if (!findMotors()) { return false; }
  }
  catch (flexiport::PortException pex)
  {
    ROS_ERROR("%s", pex.what());
    return false;
  }

  if (update_rate_ > 0)
  {
    terminate_feedback_ = false;
    feedback_thread_ = new boost::thread(boost::bind(&SerialProxy::updateMotorStates, this));
  }
  else
  {
    feedback_thread_ = NULL;
  }

  if (diagnostics_rate_ > 0)
  {
    terminate_diagnostics_ = false;
    diagnostics_thread_ = new boost::thread(boost::bind(&SerialProxy::publishDiagnosticInformation, this));
  }
  else
  {
    diagnostics_thread_ = NULL;
  }

  return true;
}

DynamixelIO* SerialProxy::getSerialPort()
{
  return dxl_io_;
}

void SerialProxy::fillMotorParameters(const DynamixelData* motor_data)
{
  int motor_id = motor_data->id;
  int model_number = motor_data->model_number;

  float voltage;
  dxl_io_->getVoltage(motor_id, voltage);

  std::stringstream ss;
  ss << "dynamixel/" << port_namespace_ << "/" << motor_id << "/";
  std::string prefix = ss.str();

  nh_.setParam(prefix + "model_number", model_number);
  nh_.setParam(prefix + "model_name", getMotorModelName(model_number));
  nh_.setParam(prefix + "min_angle", motor_data->cw_angle_limit);
  nh_.setParam(prefix + "max_angle", motor_data->ccw_angle_limit);

  double torque_per_volt = getMotorModelParams(model_number, TORQUE_PER_VOLT);
  nh_.setParam(prefix + "torque_per_volt", torque_per_volt);
  nh_.setParam(prefix + "max_torque", torque_per_volt * voltage);

  double velocity_per_volt = getMotorModelParams(model_number, VELOCITY_PER_VOLT);
  nh_.setParam(prefix + "velocity_per_volt", velocity_per_volt);
  nh_.setParam(prefix + "max_velocity", velocity_per_volt * voltage);
  nh_.setParam(prefix + "radians_second_per_encoder_tick", velocity_per_volt * voltage / DXL_MAX_VELOCITY_ENCODER);

  int encoder_resolution = (int) getMotorModelParams(model_number, ENCODER_RESOLUTION);
  double range_degrees = getMotorModelParams(model_number, RANGE_DEGREES);
  double range_radians = range_degrees * M_PI / 180.0;

  nh_.setParam(prefix + "encoder_resolution", encoder_resolution);
  nh_.setParam(prefix + "range_degrees", range_degrees);
  nh_.setParam(prefix + "range_radians", range_radians);
  nh_.setParam(prefix + "encoder_ticks_per_degree", encoder_resolution / range_degrees);
  nh_.setParam(prefix + "encoder_ticks_per_radian", encoder_resolution / range_radians);
  nh_.setParam(prefix + "degrees_per_encoder_tick", range_degrees / encoder_resolution);
  nh_.setParam(prefix + "radians_per_encoder_tick", range_radians / encoder_resolution);
}

bool SerialProxy::findMotors()
{
  ROS_INFO("%s: Pinging motor IDs %d through %d...", port_namespace_.c_str(), min_motor_id_, max_motor_id_);

  XmlRpc::XmlRpcValue val;
  std::map<int, int> counts;

  for (int motor_id = min_motor_id_; motor_id <= max_motor_id_; ++motor_id)
  {
    if (dxl_io_->ping(motor_id))
    {
      const DynamixelData* motor_data;

      if ((motor_data = dxl_io_->getCachedParameters(motor_id)) == NULL)
      {
        ROS_ERROR("Unable to retrieve cached paramaters for motor %d on port %s after successfull ping", motor_id, port_namespace_.c_str());
        continue;
      }

      counts[motor_data->model_number] += 1;
      motor_static_info_[motor_id] = motor_data;
      fillMotorParameters(motor_data);

      motors_.push_back(motor_id);
      val[motors_.size()-1] = motor_id;
    }
  }

  if (motors_.empty())
  {
    ROS_WARN("%s: No motors found.", port_namespace_.c_str());
    return false;
  }

  nh_.setParam("dynamixel/" + port_namespace_ + "/connected_ids", val);

  std::stringstream ss;
  ss << port_namespace_ << ": Found " << motors_.size() << " motors - ";

  std::map<int, int>::iterator it;
  for (it = counts.begin(); it != counts.end(); ++it)
  {
    ss << it->second << " " << getMotorModelName(it->first) << " [";
    int c = 0;

    for (size_t i = 0; i < motors_.size(); ++i)
    {
      if (motor_static_info_[motors_[i]]->model_number == it->first)
      {
        ss << motors_[i] << (++c == it->second ? "" : ", ");
      }
    }

    ss << "], ";
  }

  std::string status = ss.str();
  status.erase(status.size()-2);
  ROS_INFO("%s, initialization complete.", status.c_str());

  return true;
}

void SerialProxy::updateMotorStates()
{
  //ros::Rate rate(update_rate_);
  current_state_->motor_states.resize(motors_.size());
  dynamixel_hardware_interface::DynamixelStatus status;

  double allowed_time_usec = 1.0e6 / update_rate_;
  int sleep_time_usec = 0;

  struct timespec ts_now;
  double start_time_usec;
  double end_time_usec;

  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (terminate_feedback_) { break; }
    }

    clock_gettime(CLOCK_REALTIME, &ts_now);
    start_time_usec = ts_now.tv_sec * 1.0e6 + ts_now.tv_nsec / 1.0e3;

    for (size_t i = 0; i < motors_.size(); ++i)
    {
      int motor_id = motors_[i];

      if (dxl_io_->getFeedback(motor_id, status))
      {
        const DynamixelData* data = motor_static_info_[motor_id];
        MotorState ms;
        ms.timestamp = status.timestamp;
        ms.id = motor_id;
        ms.target_position = data->target_position;
        ms.target_velocity = data->target_velocity;
        ms.position = status.position;
        ms.velocity = status.velocity;
        ms.torque_limit = status.torque_limit;
        ms.load = status.load;
        ms.moving = status.moving;
        ms.voltage = status.voltage;
        ms.temperature = status.temperature;
        current_state_->motor_states[i] = ms;
        current_state_->motor_states[i].alive = true; // as long as we are reciving feedback the servo is considered alive
      }
      else
      {
        ROS_DEBUG("Bad feedback received from motor %d on port %s", motor_id, port_namespace_.c_str());
        current_state_->motor_states[i].alive = false; // note that data is stale
      }
    }

    motor_states_pub_.publish(current_state_);
    freq_status_.tick();

    clock_gettime(CLOCK_REALTIME, &ts_now);
    end_time_usec = ts_now.tv_sec * 1.0e6 + ts_now.tv_nsec / 1.0e3;

    sleep_time_usec = allowed_time_usec - (end_time_usec - start_time_usec);

    // do millisecond resolution sleep
    if (sleep_time_usec >= 1000)
    {
      sleep_time_usec = (sleep_time_usec / 1000) * 1000;
      usleep(sleep_time_usec);
    }

    // ros sleep somehow takes a lot more cpu time
    //rate.sleep();
  }
}

void SerialProxy::publishDiagnosticInformation()
{
  diagnostic_msgs::DiagnosticArray diag_msg;
  diagnostic_updater::DiagnosticStatusWrapper bus_status;
  ros::Rate rate(diagnostics_rate_);

  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (terminate_diagnostics_) { break; }
    }

    double error_rate = dxl_io_->read_error_count / (double) dxl_io_->read_count;

    bus_status.clear();
    bus_status.name = "Dynamixel Serial Bus (" + port_namespace_ + ")";
    bus_status.hardware_id = "Dynamixel Serial Bus on port " + port_name_;
    bus_status.add("Baud Rate", baud_rate_);
    bus_status.add("Min Motor ID", min_motor_id_);
    bus_status.add("Max Motor ID", max_motor_id_);
    bus_status.addf("Error Rate", "%0.5f", error_rate);
    bus_status.summary(bus_status.OK, "OK");

    freq_status_.run(bus_status);

    if (error_rate > 0.05)
    {
      bus_status.mergeSummary(bus_status.WARN, "Too many errors while reading/writing to/from Dynamixel bus");
    }

    diag_msg.status.clear();
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(bus_status);

    for (size_t i = 0; i < current_state_->motor_states.size(); ++i)
    {
      MotorState motor_state = current_state_->motor_states[i];
      int motor_id = motor_state.id;

      // check if current motor state was already populated by updateMotorStates thread
      if (motor_state.timestamp == 0.0) { continue; }

      const DynamixelData* data = motor_static_info_[motor_id];
      std::string mid_str = boost::lexical_cast<std::string>(motor_id);

      diagnostic_updater::DiagnosticStatusWrapper motor_status;

      motor_status.name = "Robotis Dynamixel Motor " + mid_str + " on port " + port_namespace_;
      motor_status.hardware_id = "ID " + mid_str + " on port " + port_name_;
      motor_status.add("Model Name", getMotorModelName(data->model_number).c_str());
      motor_status.addf("Firmware Version", "%d", data->firmware_version);
      motor_status.addf("Return Delay Time", "%d", data->return_delay_time);
      motor_status.addf("Minimum Voltage", "%0.1f", data->voltage_limit_low / 10.0);
      motor_status.addf("Maximum Voltage", "%0.1f", data->voltage_limit_high / 10.0);
      motor_status.addf("Maximum Torque", "%d", data->max_torque);
      motor_status.addf("Minimum Position (CW)", "%d", data->cw_angle_limit);
      motor_status.addf("Maximum Position (CCW)", "%d", data->ccw_angle_limit);
      motor_status.addf("Compliance Margin (CW)", "%d", data->cw_compliance_margin);
      motor_status.addf("Compliance Margin (CCW)", "%d", data->ccw_compliance_margin);
      motor_status.addf("Compliance Slope (CW)", "%d", data->cw_compliance_slope);
      motor_status.addf("Compliance Slope (CCW)", "%d", data->ccw_compliance_slope);
      motor_status.add("Torque Enabled", data->torque_enabled ? "True" : "False");

      motor_status.add("Moving", motor_state.moving ? "True" : "False");
      motor_status.addf("Target Position", "%d", motor_state.target_position);
      motor_status.addf("Target Velocity", "%d", motor_state.target_velocity);
      motor_status.addf("Position", "%d", motor_state.position);
      motor_status.addf("Velocity", "%d", motor_state.velocity);
      motor_status.addf("Position Error", "%d", motor_state.position - motor_state.target_position);
      motor_status.addf("Velocity Error", "%d", motor_state.velocity != 0 ? motor_state.velocity - motor_state.target_velocity : 0);
      motor_status.addf("Torque Limit", "%d", motor_state.torque_limit);
      motor_status.addf("Load", "%d", motor_state.load);
      motor_status.addf("Voltage", "%0.1f", motor_state.voltage / 10.0);
      motor_status.addf("Temperature", "%d", motor_state.temperature);

      motor_status.summary(motor_status.OK, "OK");

      if (motor_state.temperature >= error_level_temp_)
      {
        motor_status.summary(motor_status.ERROR, "Overheating");
      }
      else if (motor_state.temperature >= warn_level_temp_)
      {
        motor_status.summary(motor_status.WARN, "Very hot");
      }

      // if there was an overload or overheating error
      if (data->shutdown_error_time > 0.0)
      {
        ROS_ERROR_THROTTLE(1, "%s: %s", port_namespace_.c_str(), data->error.c_str());
        motor_status.mergeSummary(motor_status.ERROR, "Overload/Overheating error");

        // if current motor temperature is under control and
        // we just arbitrarily waited 5 seconds just for kicks
        if (motor_state.temperature < warn_level_temp_ &&
            ros::Time::now().toSec() - data->shutdown_error_time >= 5.0)
        {
          dxl_io_->resetOverloadError(motor_id);
          ROS_WARN("%s: Reset overload/overheating error on motor %d", port_namespace_.c_str(), motor_id);
        }
      }
      else if (motor_state.torque_limit == 0)
      {
        motor_status.mergeSummary(motor_status.ERROR, "Torque limit is 0");
      }

      diag_msg.status.push_back(motor_status);
    }

    diagnostics_pub_.publish(diag_msg);
    rate.sleep();
  }
}

}
