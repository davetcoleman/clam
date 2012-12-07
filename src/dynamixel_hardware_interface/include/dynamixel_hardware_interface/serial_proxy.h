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

#ifndef SERIAL_PROXY_H__
#define SERIAL_PROXY_H__

#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <boost/thread.hpp>

#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/MotorStateList.h>

#include <ros/ros.h>
#include <diagnostic_updater/update_functions.h>

namespace dynamixel_hardware_interface
{

class SerialProxy
{
public:
    SerialProxy(std::string port_name,
                std::string port_namespace,
                std::string baud_rate,
                int min_motor_id=1,
                int max_motor_id=25,
                double update_rate=10,
                double diagnostics_rate=1,
                int error_level_temp=65,
                int warn_level_temp=60);

    ~SerialProxy();

    bool connect();
    void disconnect();
    
    DynamixelIO* getSerialPort();

private:
    ros::NodeHandle nh_;

    std::string port_name_;
    std::string port_namespace_;
    std::string baud_rate_;
    int min_motor_id_;
    int max_motor_id_;
    double update_rate_;
    double diagnostics_rate_;
    int error_level_temp_;
    int warn_level_temp_;

    MotorStateListPtr current_state_;

    ros::Publisher motor_states_pub_;
    ros::Publisher diagnostics_pub_;

    boost::thread* feedback_thread_;
    boost::thread* diagnostics_thread_;

    boost::mutex terminate_mutex_;
    bool terminate_feedback_;
    bool terminate_diagnostics_;

    DynamixelIO* dxl_io_;
    std::vector<int> motors_;
    std::map<int, const DynamixelData*> motor_static_info_;

    void fillMotorParameters(const DynamixelData* motor_data);
    bool findMotors();
    void updateMotorStates();
    void publishDiagnosticInformation();
    
    diagnostic_updater::FrequencyStatus freq_status_;
};

}

#endif // SERIAL_PROXY_H__