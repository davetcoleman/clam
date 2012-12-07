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

#include <time.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include <sstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <clam/gearbox/flexiport/flexiport.h>
//#include <gearbox/src/flexiport/flexiport.h>
//#include <dynamixel_hardware_interface/flexiport.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>

namespace dynamixel_hardware_interface
{

DynamixelIO::DynamixelIO(std::string device="/dev/ttyUSB0",
                         std::string baud="1000000")
{
    std::map<std::string, std::string> options;
    options["type"] = "serial";
    //options["timeout"] = "0.0001";
    options["alwaysopen"] = "true";
    options["device"] = device;
    options["baud"] = baud;
    
    read_count = 0;
    read_error_count = 0;
    last_reset_sec = 0.0;

    pthread_mutex_init(&serial_mutex_, NULL);
    port_ = flexiport::CreatePort(options);
    
    // 100 microseconds = 0.1 milliseconds
    flexiport::Timeout t(0, 100);
    port_->SetTimeout(t);
}

DynamixelIO::~DynamixelIO()
{
    port_->Close();
    delete port_;
    pthread_mutex_destroy(&serial_mutex_);
    
    std::map<int, DynamixelData*>::iterator it;
    for (it = cache_.begin(); it != cache_.end(); ++it)
    {
        delete it->second;
    }
}

const DynamixelData* DynamixelIO::getCachedParameters(int servo_id)
{
    DynamixelData* dd = findCachedParameters(servo_id);
    if (!updateCachedParameters(servo_id, dd)) { return NULL; }
    return dd;
}

bool DynamixelIO::ping(int servo_id)
{
    // Instruction, checksum
    uint8_t length = 2;

    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint8_t checksum = 0xFF - ( (servo_id + length + DXL_PING) % 256 );

    // packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    const uint8_t packet_length = 6;
    uint8_t packet[packet_length] = { 0xFF, 0xFF, servo_id, length, DXL_PING, checksum };

    std::vector<uint8_t> response;

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet, packet_length);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);
    
    if (success)
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        updateCachedParameters(servo_id, dd);
        
        checkForErrors(servo_id, response[4], "ping");
        connected_motors_.insert(servo_id);
    }
    
    return success;
}

bool DynamixelIO::resetOverloadError(int servo_id)
{
    if (setTorqueEnable(servo_id, false))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        
        if (setTorqueLimit(servo_id, dd->max_torque))
        {
            setLed(servo_id, false);
            dd->shutdown_error_time = 0.0;
            
            return true;
        }
    }
    
    return false;
}

bool DynamixelIO::getModelNumber(int servo_id, uint16_t& model_number)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MODEL_NUMBER_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getModelNumber");
        model_number = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getFirmwareVersion(int servo_id, uint8_t& firmware_version)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_FIRMWARE_VERSION, 1, response))
    {
        checkForErrors(servo_id, response[4], "getFirmwareVersion");
        firmware_version = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getBaudRate(int servo_id, uint8_t& baud_rate)
{
    std::vector<uint8_t> response;
    
    if (read(servo_id, DXL_BAUD_RATE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getBaudRate");
        baud_rate = response[5];
        return true;
    }
    
    return false;
}

bool DynamixelIO::getReturnDelayTime(int servo_id, uint8_t& return_delay_time)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_RETURN_DELAY_TIME, 1, response))
    {
        checkForErrors(servo_id, response[4], "getReturnDelayTime");
        return_delay_time = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getAngleLimits(int servo_id, uint16_t& cw_angle_limit, uint16_t& ccw_angle_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_ANGLE_LIMIT_L, 4, response))
    {
        checkForErrors(servo_id, response[4], "getAngleLimits");
        cw_angle_limit = response[5] + (response[6] << 8);
        ccw_angle_limit = response[7] + (response[8] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getCWAngleLimit(int servo_id, uint16_t& cw_angle)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_ANGLE_LIMIT_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getCWAngleLimit");
        cw_angle = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getCCWAngleLimit(int servo_id, uint16_t& ccw_angle)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CCW_ANGLE_LIMIT_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getCCWAngleLimit");
        ccw_angle = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2, response))
    {
        checkForErrors(servo_id, response[4], "getVoltageLimits");
        min_voltage_limit = response[5] / 10.0;
        max_voltage_limit = response[6] / 10.0;
        return true;
    }

    return false;
}

bool DynamixelIO::getMinVoltageLimit(int servo_id, float& min_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getMinVoltageLimit");
        min_voltage_limit = response[5] / 10.0;
        return true;
    }

    return false;
}

bool DynamixelIO::getMaxVoltageLimit(int servo_id, float& max_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_UP_LIMIT_VOLTAGE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getMaxVoltageLimit");
        max_voltage_limit = response[5] / 10.0;
        return true;
    }

    return false;
}

bool DynamixelIO::getTemperatureLimit(int servo_id, uint8_t& max_temperature)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_LIMIT_TEMPERATURE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getTemperatureLimit");
        max_temperature = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getMaxTorque(int servo_id, uint16_t& max_torque)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MAX_TORQUE_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getMaximumTorque");
        max_torque = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getAlarmLed(int servo_id, uint8_t& alarm_led)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_ALARM_LED, 1, response))
    {
        checkForErrors(servo_id, response[4], "getAlarmLed");
        alarm_led = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getAlarmShutdown(int servo_id, uint8_t& alarm_shutdown)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_ALARM_SHUTDOWN, 1, response))
    {
        checkForErrors(servo_id, response[4], "getAlarmShutdown");
        alarm_shutdown = response[5];
        return true;
    }

    return false;
}


bool DynamixelIO::getTorqueEnable(int servo_id, bool& torque_enabled)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_TORQUE_ENABLE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getTorqueEnable");
        torque_enabled = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getLedStatus(int servo_id, bool& led_enabled)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_LED, 1, response))
    {
        checkForErrors(servo_id, response[4], "getLedStatus");
        led_enabled = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getComplianceMargins(int servo_id, uint8_t& cw_compliance_margin, uint8_t& ccw_compliance_margin)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_COMPLIANCE_MARGIN, 2, response))
    {
        checkForErrors(servo_id, response[4], "getComplianceMargins");
        cw_compliance_margin = response[5];
        ccw_compliance_margin = response[6];
        return true;
    }

    return false;
}

bool DynamixelIO::getCWComplianceMargin(int servo_id, uint8_t& cw_compliance_margin)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_COMPLIANCE_MARGIN, 1, response))
    {
        checkForErrors(servo_id, response[4], "getCWComplianceMargin");
        cw_compliance_margin = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getCCWComplianceMargin(int servo_id, uint8_t& ccw_compliance_margin)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CCW_COMPLIANCE_MARGIN, 1, response))
    {
        checkForErrors(servo_id, response[4], "getCCWComplianceMargin");
        ccw_compliance_margin = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getComplianceSlopes(int servo_id, uint8_t& cw_compliance_slope, uint8_t& ccw_compliance_slope)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_COMPLIANCE_SLOPE, 2, response))
    {
        checkForErrors(servo_id, response[4], "getComplianceSlopes");
        cw_compliance_slope = response[5];
        ccw_compliance_slope = response[6];
        return true;
    }

    return false;
}

bool DynamixelIO::getCWComplianceSlope(int servo_id, uint8_t& cw_compliance_slope)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CW_COMPLIANCE_SLOPE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getCWComplianceSlope");
        cw_compliance_slope = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getCCWComplianceSlope(int servo_id, uint8_t& ccw_compliance_slope)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_CCW_COMPLIANCE_SLOPE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getCCWComplianceSlope");
        ccw_compliance_slope = response[5];
        return true;
    }

    return false;
}


bool DynamixelIO::getTargetPosition(int servo_id, uint16_t& target_position)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_GOAL_POSITION_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getTargetPosition");
        target_position = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getTargetVelocity(int servo_id, int16_t& target_velocity)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_GOAL_SPEED_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getTargetVelocity");
        target_velocity = response[5] + (response[6] << 8);
        int direction = (target_velocity & (1 << 10)) == 0 ? 1 : -1;
        target_velocity = direction * (target_velocity & DXL_MAX_VELOCITY_ENCODER);
        return true;
    }

    return false;
}

bool DynamixelIO::getTorqueLimit(int servo_id, uint16_t& torque_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_TORQUE_LIMIT_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getTorqueLimit");
        torque_limit = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}


bool DynamixelIO::getPosition(int servo_id, uint16_t& position)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_POSITION_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getPosition");
        position = response[5] + (response[6] << 8);
        return true;
    }

    return false;
}

bool DynamixelIO::getVelocity(int servo_id, int16_t& velocity)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_SPEED_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getVelocity");
        velocity = response[5] + (response[6] << 8);
        int direction = (velocity & (1 << 10)) == 0 ? 1 : -1;
        velocity = direction * (velocity & DXL_MAX_VELOCITY_ENCODER);
        return true;
    }

    return false;
}

bool DynamixelIO::getLoad(int servo_id, int16_t& load)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_LOAD_L, 2, response))
    {
        checkForErrors(servo_id, response[4], "getLoad");
        load = response[5] + (response[6] << 8);
        int direction = (load & (1 << 10)) == 0 ? 1 : -1;
        load = direction * (load & DXL_MAX_LOAD_ENCODER);
        return true;
    }

    return false;
}

bool DynamixelIO::getVoltage(int servo_id, float& voltage)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_VOLTAGE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getVoltage");
        voltage = response[5] / 10.0;
        return true;
    }

    return false;
}

bool DynamixelIO::getTemperature(int servo_id, uint8_t& temperature)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_TEMPERATURE, 1, response))
    {
        checkForErrors(servo_id, response[4], "getTemperature");
        temperature = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getMoving(int servo_id, bool& is_moving)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_LED, 1, response))
    {
        checkForErrors(servo_id, response[4], "getMoving");
        is_moving = response[5];
        return true;
    }

    return false;
}

bool DynamixelIO::getFeedback(int servo_id, DynamixelStatus& status)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_TORQUE_LIMIT_L, 13, response))
    {
        struct timespec ts_now;
        clock_gettime(CLOCK_REALTIME, &ts_now);
        double timestamp = ts_now.tv_sec + ts_now.tv_nsec / 1.0e9;

        checkForErrors(servo_id, response[4], "getFeedback");

        if (response.size() == 19)
        {
            int offset = 5;
            
            uint16_t torque_limit = response[offset+0] + (response[offset+1] << 8);
            uint16_t position = response[offset+2] + (response[offset+3] << 8);

            int16_t velocity = response[offset+4] + (response[offset+5] << 8);
            int direction = (velocity & (1 << 10)) == 0 ? 1 : -1;
            velocity = direction * (velocity & DXL_MAX_VELOCITY_ENCODER);

            int16_t load = response[offset+6] + (response[offset+7] << 8);
            direction = (load & (1 << 10)) == 0 ? 1 : -1;
            load = direction * (load & DXL_MAX_LOAD_ENCODER);

            uint8_t voltage = response[offset+8];
            uint8_t temperature = response[offset+9];
            bool moving = response[offset+12];

            status.timestamp = timestamp;
            status.torque_limit = torque_limit;
            status.position = position;
            status.velocity = velocity;
            status.load = load;
            status.voltage = voltage;
            status.temperature = temperature;
            status.moving = moving;

            return true;
        }
    }

    return false;
}



/************************ SETTERS **************************/

bool DynamixelIO::setId(int servo_id, uint8_t id)
{
    std::vector<uint8_t> data;
    data.push_back(id);
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_ID, data, response))
    {
        checkForErrors(servo_id, response[4], "setId");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setBaudRate(int servo_id, uint8_t baud_rate)
{
    std::vector<uint8_t> data;
    data.push_back(baud_rate);
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_BAUD_RATE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->baud_rate = baud_rate;
        
        checkForErrors(servo_id, response[4], "setBaudRate");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setReturnDelayTime(int servo_id, uint8_t return_delay_time)
{
    std::vector<uint8_t> data;
    data.push_back(return_delay_time);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_RETURN_DELAY_TIME, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->return_delay_time = return_delay_time;
        
        checkForErrors(servo_id, response[4], "setReturnDelayTime");
        return true;
    }

    return false;
}

bool DynamixelIO::setAngleLimits(int servo_id, uint16_t cw_angle, uint16_t ccw_angle)
{
    std::vector<uint8_t> data;
    data.push_back(cw_angle % 256);     // lo_byte
    data.push_back(cw_angle >> 8);      // hi_byte
    data.push_back(ccw_angle % 256);    // lo_byte
    data.push_back(ccw_angle >> 8);     // hi_byte
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_CW_ANGLE_LIMIT_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_angle_limit = cw_angle;
        dd->ccw_angle_limit = ccw_angle;
        
        checkForErrors(servo_id, response[4], "setAngleLimits");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setCWAngleLimit(int servo_id, uint16_t cw_angle)
{
    std::vector<uint8_t> data;
    data.push_back(cw_angle % 256); // lo_byte
    data.push_back(cw_angle >> 8);  // hi_byte
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_CW_ANGLE_LIMIT_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_angle_limit = cw_angle;
        
        checkForErrors(servo_id, response[4], "setCWAngleLimit");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setCCWAngleLimit(int servo_id, uint16_t ccw_angle)
{
    std::vector<uint8_t> data;
    data.push_back(ccw_angle % 256); // lo_byte
    data.push_back(ccw_angle >> 8);  // hi_byte
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_CCW_ANGLE_LIMIT_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->ccw_angle_limit = ccw_angle;
        
        checkForErrors(servo_id, response[4], "setCCWAngleLimit");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setVoltageLimits(int servo_id, float min_voltage_limit, float max_voltage_limit)
{
    uint8_t min_voltage = min_voltage_limit * 10;
    uint8_t max_voltage = max_voltage_limit * 10;
    
    std::vector<uint8_t> data;
    data.push_back(min_voltage);
    data.push_back(max_voltage);
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->voltage_limit_low = min_voltage;
        dd->voltage_limit_high = max_voltage;
        
        checkForErrors(servo_id, response[4], "setVoltageLimits");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setMinVoltageLimit(int servo_id, float min_voltage_limit)
{
    uint8_t min_voltage = min_voltage_limit * 10;
    
    std::vector<uint8_t> data;
    data.push_back(min_voltage);
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->voltage_limit_low = min_voltage;
        
        checkForErrors(servo_id, response[4], "setMinVoltageLimit");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setMaxVoltageLimit(int servo_id, float max_voltage_limit)
{
    uint8_t max_voltage = max_voltage_limit * 10;
    
    std::vector<uint8_t> data;
    data.push_back(max_voltage);
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_UP_LIMIT_VOLTAGE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->voltage_limit_high = max_voltage;
        
        checkForErrors(servo_id, response[4], "setMaxVoltageLimit");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setTemperatureLimit(int servo_id, uint8_t max_temperature)
{
    std::vector<uint8_t> data;
    data.push_back(max_temperature);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_LIMIT_TEMPERATURE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->temperature_limit = max_temperature;
        
        checkForErrors(servo_id, response[4], "setTemperatureLimit");
        return true;
    }

    return false;
}

bool DynamixelIO::setMaxTorque(int servo_id, uint16_t max_torque)
{
    std::vector<uint8_t> data;
    data.push_back(max_torque % 256); // lo_byte
    data.push_back(max_torque >> 8);  // hi_byte
    
    std::vector<uint8_t> response;
    
    if (write(servo_id, DXL_MAX_TORQUE_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->max_torque = max_torque;
        
        checkForErrors(servo_id, response[4], "setMaxTorque");
        return true;
    }
    
    return false;
}

bool DynamixelIO::setAlarmLed(int servo_id, uint8_t alarm_led)
{
    std::vector<uint8_t> data;
    data.push_back(alarm_led);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_ALARM_LED, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->alarm_led = alarm_led;
        
        checkForErrors(servo_id, response[4], "setAlarmLed");
        return true;
    }

    return false;
}

bool DynamixelIO::setAlarmShutdown(int servo_id, uint8_t alarm_shutdown)
{
    std::vector<uint8_t> data;
    data.push_back(alarm_shutdown);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_ALARM_SHUTDOWN, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->alarm_shutdown = alarm_shutdown;
        
        checkForErrors(servo_id, response[4], "setAlarmShutdown");
        return true;
    }

    return false;
}

bool DynamixelIO::setTorqueEnable(int servo_id, bool on)
{
    std::vector<uint8_t> data;
    data.push_back(on);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_TORQUE_ENABLE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->torque_enabled = on;
        
        checkForErrors(servo_id, response[4], "setTorqueEnable");
        return true;
    }

    return false;
}

bool DynamixelIO::setLed(int servo_id, bool on)
{
    std::vector<uint8_t> data;
    data.push_back(on);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_LED, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->led = on;
        
        checkForErrors(servo_id, response[4], "setLed");
        return true;
    }

    return false;
}

bool DynamixelIO::setComplianceMargins(int servo_id, uint8_t cw_margin, uint8_t ccw_margin)
{
    std::vector<uint8_t> data;
    data.push_back(cw_margin);
    data.push_back(ccw_margin);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CW_COMPLIANCE_MARGIN, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_compliance_margin = cw_margin;
        dd->ccw_compliance_margin = ccw_margin;
        
        checkForErrors(servo_id, response[4], "setComplianceMargins");
        return true;
    }

    return false;
}

bool DynamixelIO::setCWComplianceMargin(int servo_id, uint8_t cw_margin)
{
    std::vector<uint8_t> data;
    data.push_back(cw_margin);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CW_COMPLIANCE_MARGIN, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_compliance_margin = cw_margin;
        
        checkForErrors(servo_id, response[4], "setCWComplianceMargin");
        return true;
    }

    return false;
}

bool DynamixelIO::setCCWComplianceMargin(int servo_id, uint8_t ccw_margin)
{
    std::vector<uint8_t> data;
    data.push_back(ccw_margin);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CCW_COMPLIANCE_MARGIN, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->ccw_compliance_margin = ccw_margin;
        
        checkForErrors(servo_id, response[4], "setCCWComplianceMargin");
        return true;
    }

    return false;
}

bool DynamixelIO::setComplianceSlopes(int servo_id, uint8_t cw_slope, uint8_t ccw_slope)
{
    std::vector<uint8_t> data;
    data.push_back(cw_slope);
    data.push_back(ccw_slope);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CW_COMPLIANCE_SLOPE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_compliance_slope = cw_slope;
        dd->ccw_compliance_slope = ccw_slope;
        
        checkForErrors(servo_id, response[4], "setComplianceSlopes");
        return true;
    }

    return false;
}

bool DynamixelIO::setCWComplianceSlope(int servo_id, uint8_t cw_slope)
{
    std::vector<uint8_t> data;
    data.push_back(cw_slope);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CW_COMPLIANCE_SLOPE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->cw_compliance_slope = cw_slope;
        
        checkForErrors(servo_id, response[4], "setCWComplianceSlope");
        return true;
    }

    return false;
}

bool DynamixelIO::setCCWComplianceSlope(int servo_id, uint8_t ccw_slope)
{
    std::vector<uint8_t> data;
    data.push_back(ccw_slope);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_CCW_COMPLIANCE_SLOPE, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->ccw_compliance_slope = ccw_slope;
        
        checkForErrors(servo_id, response[4], "setCCWComplianceSlope");
        return true;
    }

    return false;
}

bool DynamixelIO::setPosition(int servo_id, uint16_t position)
{
    std::vector<uint8_t> data;
    data.push_back(position % 256); // lo_byte
    data.push_back(position >> 8);  // hi_byte

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_GOAL_POSITION_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->target_position = position;
        dd->torque_enabled = true;
        
        checkForErrors(servo_id, response[4], "setPosition");
        return true;
    }

    return false;
}


bool DynamixelIO::setVelocity(int servo_id, int16_t velocity)
{
    std::vector<uint8_t> data;

    if (velocity >= 0)
    {
        data.push_back(velocity % 256); // lo_byte
        data.push_back(velocity >> 8);  // hi_byte
    }
    else
    {
        data.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) % 256); // lo_byte
        data.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) >> 8);  // hi_byte
    }

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_GOAL_SPEED_L, data, response))
    {
        DynamixelData* dd = findCachedParameters(servo_id);
        dd->target_velocity = velocity;
        dd->torque_enabled = true;

        checkForErrors(servo_id, response[4], "setVelocity");
        return true;
    }

    return false;
}

bool DynamixelIO::setTorqueLimit(int servo_id, uint16_t torque_limit)
{
    std::vector<uint8_t> data;
    data.push_back(torque_limit % 256); // lo_byte
    data.push_back(torque_limit >> 8);  // hi_byte

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_TORQUE_LIMIT_L, data, response))
    {
        checkForErrors(servo_id, response[4], "setTorqueLimit");
        return true;
    }

    return false;
}


bool DynamixelIO::setMultiPosition(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        int position = value_pairs[i][1];

        DynamixelData* dd = findCachedParameters(motor_id);
        dd->target_position = position;
        dd->torque_enabled = true;
        
        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);                 // servo id
        value_pair.push_back(position % 256);           // lo_byte
        value_pair.push_back(position >> 8);            // hi_byte

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_GOAL_POSITION_L, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiVelocity(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        int velocity = value_pairs[i][1];

        DynamixelData* dd = findCachedParameters(motor_id);
        dd->target_velocity = velocity;
        dd->torque_enabled = true;
        
        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);             // servo id

        if (velocity >= 0)
        {
            value_pair.push_back(velocity % 256);   // lo_byte
            value_pair.push_back(velocity >> 8);    // hi_byte
        }
        else
        {
            value_pair.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) % 256);  // lo_byte
            value_pair.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) >> 8);   // hi_byte
        }

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_GOAL_SPEED_L, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiPositionVelocity(std::vector<std::vector<int> > value_tuples)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_tuples.size(); ++i)
    {
        int motor_id = value_tuples[i][0];
        int position = value_tuples[i][1];
        int velocity = value_tuples[i][2];

        DynamixelData* dd = findCachedParameters(motor_id);
        dd->target_position = position;
        dd->target_velocity = velocity;
        dd->torque_enabled = true;

        std::vector<uint8_t> vals;

        vals.push_back(motor_id);     // servo id
        vals.push_back(position % 256);         // lo_byte
        vals.push_back(position >> 8);          // hi_byte

        if (velocity >= 0)
        {
            vals.push_back(velocity % 256);     // lo_byte
            vals.push_back(velocity >> 8);      // hi_byte
        }
        else
        {
            vals.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) % 256);    // lo_byte
            vals.push_back((DXL_MAX_VELOCITY_ENCODER - velocity) >> 8);     // hi_byte
        }

        data.push_back(vals);
    }

    if (syncWrite(DXL_GOAL_POSITION_L, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiComplianceMargins(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;
    
    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        int cw_margin = value_pairs[i][1];
        int ccw_margin = value_pairs[i][2];
        
        DynamixelData* dd = findCachedParameters(motor_id);
        dd->cw_compliance_margin = cw_margin;
        dd->ccw_compliance_margin = ccw_margin;
        
        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);         // servo id
        value_pair.push_back(cw_margin);        // cw_compliance_margin
        value_pair.push_back(ccw_margin);       // ccw_compliance_margin
        
        data.push_back(value_pair);
    }
    
    if (syncWrite(DXL_CW_COMPLIANCE_MARGIN, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiComplianceSlopes(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;
    
    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        int cw_slope = value_pairs[i][1];
        int ccw_slope = value_pairs[i][2];
        
        DynamixelData* dd = findCachedParameters(motor_id);
        dd->cw_compliance_slope = cw_slope;
        dd->ccw_compliance_slope = ccw_slope;
        
        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);     // servo id
        value_pair.push_back(cw_slope);     // cw_compliance_slope
        value_pair.push_back(ccw_slope);    // ccw_compliance_slope
        
        data.push_back(value_pair);
    }
    
    if (syncWrite(DXL_CW_COMPLIANCE_SLOPE, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiTorqueEnabled(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;
    
    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        bool torque_enabled = value_pairs[i][1];
        
        DynamixelData* dd = findCachedParameters(motor_id);
        dd->torque_enabled = torque_enabled;
        
        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);         // servo id
        value_pair.push_back(torque_enabled);   // torque_enabled
        
        data.push_back(value_pair);
    }
    
    if (syncWrite(DXL_TORQUE_ENABLE, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiTorqueLimit(std::vector<std::vector<int> > value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        std::vector<uint8_t> value_pair;

        int torque_limit = value_pairs[i][1];

        value_pair.push_back(value_pairs[i][0]);        // servo id
        value_pair.push_back(torque_limit % 256);       // lo_byte
        value_pair.push_back(torque_limit >> 8);        // hi_byte

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_TORQUE_LIMIT_L, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::setMultiValues(std::vector<std::map<std::string, int> > value_maps)
{
    std::vector<std::vector<uint8_t> > data;
    
    for (size_t i = 0; i < value_maps.size(); ++i)
    {
        std::map<std::string, int> m = value_maps[i];
        std::map<std::string, int>::const_iterator it;
        
        it = m.find("id");
        if (it == m.end()) { return false; }
        
        int id = it->second;
        const DynamixelData* cache = getCachedParameters(id);
        if (cache == NULL) { return false; }
        
        bool     torque_enabled = cache->torque_enabled;
        uint8_t  led = cache->led;
        uint8_t  cw_compliance_margin = cache->cw_compliance_margin;
        uint8_t  ccw_compliance_margin = cache->ccw_compliance_margin;
        uint8_t  cw_compliance_slope = cache->cw_compliance_slope;
        uint8_t  ccw_compliance_slope = cache->ccw_compliance_slope;
        uint16_t target_position = cache->target_position;
        int16_t  target_velocity = cache->target_velocity;
        
        it = m.find("torque_enabled");
        if (it != m.end()) { torque_enabled = it->second; }
        
        it = m.find("cw_compliance_margin");
        if (it != m.end()) { cw_compliance_margin = it->second; }
        
        it = m.find("ccw_compliance_margin");
        if (it != m.end()) { ccw_compliance_margin = it->second; }
        
        it = m.find("cw_compliance_slope");
        if (it != m.end()) { cw_compliance_slope = it->second; }
        
        it = m.find("ccw_compliance_slope");
        if (it != m.end()) { ccw_compliance_slope = it->second; }
        
        it = m.find("target_position");
        if (it != m.end()) { target_position = it->second; }
        
        it = m.find("target_velocity");
        if (it != m.end()) { target_velocity = it->second; }
        
        std::vector<uint8_t> vals;
        
        vals.push_back(id);
        
        vals.push_back(torque_enabled);
        vals.push_back(led);
        vals.push_back(cw_compliance_margin);
        vals.push_back(ccw_compliance_margin);
        vals.push_back(cw_compliance_slope);
        vals.push_back(ccw_compliance_slope);
        
        vals.push_back(target_position % 256);         // lo_byte
        vals.push_back(target_position >> 8);          // hi_byte
        
        if (target_velocity >= 0)
        {
            vals.push_back(target_velocity % 256);     // lo_byte
            vals.push_back(target_velocity >> 8);      // hi_byte
        }
        else
        {
            vals.push_back((DXL_MAX_VELOCITY_ENCODER - target_velocity) % 256);    // lo_byte
            vals.push_back((DXL_MAX_VELOCITY_ENCODER - target_velocity) >> 8);     // hi_byte
        }
        
        data.push_back(vals);
    }

    if (syncWrite(DXL_TORQUE_ENABLE, data)) { return true; }
    else { return false; }
}

bool DynamixelIO::updateCachedParameters(int servo_id, DynamixelData* data)
{
    std::vector<uint8_t> response;
    if (read(servo_id, DXL_MODEL_NUMBER_L, 34, response))
    {
        uint8_t byte_num = 5;
        
        data->model_number = response[byte_num+0] + (response[byte_num+1] << 8);
        data->firmware_version = response[byte_num+2];
        data->id = response[byte_num+3];
        data->baud_rate = response[byte_num+4];
        data->return_delay_time = response[byte_num+5];
        data->cw_angle_limit = response[byte_num+6] + (response[byte_num+7] << 8);
        data->ccw_angle_limit = response[byte_num+8] + (response[byte_num+9] << 8);
        data->drive_mode = response[byte_num+10];
        data->temperature_limit = response[byte_num+11];
        data->voltage_limit_low = response[byte_num+12];
        data->voltage_limit_high = response[byte_num+13];
        data->max_torque = response[byte_num+14] + (response[byte_num+15] << 8);
        data->return_level = response[byte_num+16];
        data->alarm_led = response[byte_num+17];
        data->alarm_shutdown = response[byte_num+18];
        data->torque_enabled = response[byte_num+24];
        data->led = response[byte_num+25];
        data->cw_compliance_margin = response[byte_num+26];
        data->ccw_compliance_margin = response[byte_num+27];
        data->cw_compliance_slope = response[byte_num+28];
        data->ccw_compliance_slope = response[byte_num+29];
        data->target_position = response[byte_num+30] + (response[byte_num+31] << 8);
        
        int16_t target_velocity = response[byte_num+32] + (response[byte_num+33] << 8);
        int direction = (target_velocity & (1 << 10)) == 0 ? 1 : -1;
        target_velocity = direction * (target_velocity & DXL_MAX_VELOCITY_ENCODER);
        data->target_velocity = target_velocity;
        
        return true;
    }

    return false;
}

void DynamixelIO::checkForErrors(int servo_id, uint8_t error_code, std::string command_failed)
{
    DynamixelData* dd = findCachedParameters(servo_id);
    
    if (error_code == DXL_NO_ERROR)
    {
        dd->shutdown_error_time = 0.0;
        return;        
    }
    
    std::vector<std::string> error_msgs;

    if ((error_code & DXL_OVERHEATING_ERROR) != 0)
    {
        if (dd->shutdown_error_time <= 0.0)
        {
            struct timespec ts_now;
            clock_gettime(CLOCK_REALTIME, &ts_now);
            dd->shutdown_error_time = ts_now.tv_sec + ts_now.tv_nsec / 1.0e9;
        }
        
        error_msgs.push_back("Overheating Error");
    }
    
    if ((error_code & DXL_OVERLOAD_ERROR) != 0)
    {
        if (dd->shutdown_error_time <= 0.0)
        {
            struct timespec ts_now;
            clock_gettime(CLOCK_REALTIME, &ts_now);
            dd->shutdown_error_time = ts_now.tv_sec + ts_now.tv_nsec / 1.0e9;
        }
        
        error_msgs.push_back("Overload Error");
    }
    
    if ((error_code & DXL_INPUT_VOLTAGE_ERROR) != 0) { error_msgs.push_back("Input Voltage Error"); }
    if ((error_code & DXL_ANGLE_LIMIT_ERROR) != 0)   { error_msgs.push_back("Angle Limit Error"); }
    if ((error_code & DXL_RANGE_ERROR) != 0)         { error_msgs.push_back("Range Error"); }
    if ((error_code & DXL_CHECKSUM_ERROR) != 0)      { error_msgs.push_back("Checksum Error"); }
    if ((error_code & DXL_INSTRUCTION_ERROR) != 0)   { error_msgs.push_back("Instruction Error"); }
    
    std::stringstream m;
    m << "Detected error condition [";

    for (size_t i = 0; i < error_msgs.size(); ++i)
    {
        m << error_msgs[i] << (i != error_msgs.size()-1 ? ", " : "");
    }
    
    m << "] during " << command_failed << " command on servo #" << servo_id; 
    dd->error = m.str();
    updateCachedParameters(servo_id, dd);
}

bool DynamixelIO::read(int servo_id,
                       int address,
                       int size,
                       std::vector<uint8_t>& response)
{
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    uint8_t length = 4;

    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint8_t checksum = 0xFF - ( (servo_id + length + DXL_READ_DATA + address + size) % 256 );

    // packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    uint8_t packet[8] = { 0xFF, 0xFF, servo_id, length, DXL_READ_DATA, address, size, checksum };

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet, 8);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelIO::write(int servo_id,
                        int address,
                        const std::vector<uint8_t>& data,
                        std::vector<uint8_t>& response)
{
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    // instruction, address, len(data), checksum
    uint8_t length = 3 + data.size();

    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint32_t sum = 0;

    for (uint32_t i = 0; i < data.size(); ++i)
    {
        sum += data[i];
    }

    uint8_t checksum = 0xFF - ( (servo_id + length + DXL_WRITE_DATA + address + sum) % 256 );

    // packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    uint8_t packetLength = 4 + length;
    uint8_t packet[packetLength];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = DXL_WRITE_DATA;
    packet[5] = address;

    for (uint8_t i = 0; i < data.size(); ++i)
    {
        packet[6+i] = data[i];
    }

    packet[packetLength-1] = checksum;

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet, packetLength);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelIO::syncWrite(int address,
                            const std::vector<std::vector<uint8_t> >& data)
{
    // data = ( (id, byte1, byte2... ), (id, byte1, byte2...), ... )

    // Instruction, address, length, checksum
    uint8_t length = 4;

    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint32_t sum = 0;
    uint8_t servo_length = 0;

    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            sum += data[i][j];
        }

        length += data[i].size();
        servo_length = data[i].size() - 1;
    }

    uint8_t checksum = 0xFF - ( (DXL_BROADCAST + length + DXL_SYNC_WRITE + address + servo_length + sum) % 256 );

    // packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    int packet_length = 4 + length;
    uint8_t packet[packet_length];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = DXL_BROADCAST;
    packet[3] = length;
    packet[4] = DXL_SYNC_WRITE;
    packet[5] = address;
    packet[6] = servo_length;

    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            packet[7+i*data[i].size()+j] = data[i][j];
        }
    }

    packet[packet_length-1] = checksum;

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet, packet_length);
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelIO::waitForBytes(ssize_t n_bytes, uint16_t timeout_ms)
{
    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    double start_time_ms = ts_now.tv_sec * 1.0e3 + ts_now.tv_nsec / 1.0e6;

    // wait for response packet from the motor
    while (port_->BytesAvailableWait() < n_bytes)
    {
        clock_gettime(CLOCK_REALTIME, &ts_now);
        double current_time_ms = ts_now.tv_sec * 1.0e3 + ts_now.tv_nsec / 1.0e6;

        if (current_time_ms - start_time_ms > timeout_ms)
        {
            //printf("waitForBytes timed out trying to read %zd bytes in less than %dms\n", n_bytes, timeout_ms);
            return false;
        }
    }

    return true;
}

bool DynamixelIO::writePacket(const void* const buffer, size_t count)
{
    port_->Flush();
    return (port_->Write(buffer, count) == (ssize_t) count);
}

bool DynamixelIO::readResponse(std::vector<uint8_t>& response)
{
    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    double current_time_sec = ts_now.tv_sec + ts_now.tv_nsec / 1.0e9;
    
    if (current_time_sec - last_reset_sec > 20)
    {
        read_count = 0;
        read_error_count = 0;
        last_reset_sec = current_time_sec;
    }
    
    ++read_count;
    
    static const uint16_t timeout_ms = 50;
    
    uint8_t buffer[1024];
    response.clear();

    // wait until we receive the header bytes and read them
    if (!waitForBytes(4, timeout_ms) || port_->Read(buffer, 4) != 4)
    {
        ++read_error_count;
        return false;
    }
    
    if (buffer[0] == 0xFF && buffer[1] == 0xFF)
    {
        response.push_back(buffer[0]);  // 0xFF
        response.push_back(buffer[1]);  // 0xFF
        response.push_back(buffer[2]);  // ID
        uint8_t n_bytes = buffer[3];    // Length
        response.push_back(n_bytes);
        
        // wait for and read the rest of response bytes
        if (!waitForBytes(n_bytes, timeout_ms) || port_->Read(buffer, n_bytes) != n_bytes)
        {
            ++read_error_count;
            response.clear();
            return false;
        }
        
        for (int i = 0; i < n_bytes; ++i)
        {
            response.push_back(buffer[i]);
        }

        // verify checksum
        uint8_t checksum = 0xFF;
        uint32_t sum = 0;

        for (uint32_t i = 2; i < response.size() - 1; ++i)
        {
            sum += response[i];
        }

        checksum -= sum % 256;
        
        if (checksum != response.back())
        {
            ++read_error_count;
            return false;
        }

        return true;
    }

    return false;
}

}
