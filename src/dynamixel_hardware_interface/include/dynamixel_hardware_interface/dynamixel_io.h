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

#ifndef DYNAMIXEL_IO_H__
#define DYNAMIXEL_IO_H__

#include <pthread.h>
#include <stdint.h>

#include <set>
#include <map>
#include <string>
#include <vector>

#include <clam/gearbox/flexiport/port.h>

namespace dynamixel_hardware_interface
{

typedef struct DynamixelDataStruct
{
    uint16_t model_number;
    uint8_t  firmware_version;
    uint8_t  id;
    uint8_t  baud_rate;
    uint8_t  return_delay_time;
    uint16_t cw_angle_limit;
    uint16_t ccw_angle_limit;
    uint8_t  drive_mode;
    uint8_t  temperature_limit;
    uint8_t  voltage_limit_low;
    uint8_t  voltage_limit_high;
    uint16_t max_torque;
    uint8_t  return_level;
    uint8_t  alarm_led;
    uint8_t  alarm_shutdown;
    bool     torque_enabled;
    uint8_t  led;
    uint8_t  cw_compliance_margin;
    uint8_t  ccw_compliance_margin;
    uint8_t  cw_compliance_slope;
    uint8_t  ccw_compliance_slope;
    uint16_t target_position;
    int16_t  target_velocity;
    
    double   shutdown_error_time;
    std::string error;
    
} DynamixelData;

typedef struct DynamixelStatusStruct
{
    double timestamp;
    
    uint16_t torque_limit;
    uint16_t position;
    int16_t  velocity;
    int16_t  load;
    uint8_t  voltage;
    uint8_t  temperature;
    bool     moving;

} DynamixelStatus;


class DynamixelIO
{
public:
    DynamixelIO(std::string device, std::string baud);
    ~DynamixelIO();

    long long unsigned int read_error_count;
    long long unsigned int read_count;
    double last_reset_sec;
    
    const DynamixelData* getCachedParameters(int servo_id);
    
    bool ping(int servo_id);
    bool resetOverloadError(int servo_id);
    
    // ****************************** GETTERS ******************************** //
    bool getModelNumber(int servo_id, uint16_t& model_number);
    bool getFirmwareVersion(int servo_id, uint8_t& firmware_version);
    bool getBaudRate(int servo_id, uint8_t& baud_rate);
    bool getReturnDelayTime(int servo_id, uint8_t& return_delay_time);
    
    bool getAngleLimits(int servo_id, uint16_t& cw_angle_limit, uint16_t& ccw_angle_limit);
    bool getCWAngleLimit(int servo_id, uint16_t& cw_angle);
    bool getCCWAngleLimit(int servo_id, uint16_t& ccw_angle);
    
    bool getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit);
    bool getMinVoltageLimit(int servo_id, float& min_voltage_limit);
    bool getMaxVoltageLimit(int servo_id, float& max_voltage_limit);
    
    bool getTemperatureLimit(int servo_id, uint8_t& max_temperature);
    bool getMaxTorque(int servo_id, uint16_t& max_torque);
    bool getAlarmLed(int servo_id, uint8_t& alarm_led);
    bool getAlarmShutdown(int servo_id, uint8_t& alarm_shutdown);
    bool getTorqueEnable(int servo_id, bool& torque_enabled);
    bool getLedStatus(int servo_id, bool& led_enabled);
    
    bool getComplianceMargins(int servo_id, uint8_t& cw_compliance_margin, uint8_t& ccw_compliance_margin);
    bool getCWComplianceMargin(int servo_id, uint8_t& cw_compliance_margin);
    bool getCCWComplianceMargin(int servo_id, uint8_t& ccw_compliance_margin);
    
    bool getComplianceSlopes(int servo_id, uint8_t& cw_compliance_slope, uint8_t& ccw_compliance_slope);
    bool getCWComplianceSlope(int servo_id, uint8_t& cw_compliance_slope);
    bool getCCWComplianceSlope(int servo_id, uint8_t& ccw_compliance_slope);
    
    bool getTargetPosition(int servo_id, uint16_t& target_position);
    bool getTargetVelocity(int servo_id, int16_t& target_velocity);
    bool getTorqueLimit(int servo_id, uint16_t& torque_limit);

    bool getPosition(int servo_id, uint16_t& position);
    bool getVelocity(int servo_id, int16_t& velocity);
    bool getLoad(int servo_id, int16_t& load);
    bool getVoltage(int servo_id, float& voltage);
    bool getTemperature(int servo_id, uint8_t& temperature);
    bool getMoving(int servo_id, bool& is_moving);
    
    bool getFeedback(int servo_id, DynamixelStatus& status);

    // ****************************** SETTERS ******************************** //
    bool setId(int servo_id, uint8_t id);
    bool setBaudRate(int servo_id, uint8_t baud_rate);
    bool setReturnDelayTime(int servo_id, uint8_t return_delay_time);
    
    bool setAngleLimits(int servo_id, uint16_t cw_angle, uint16_t ccw_angle);
    bool setCWAngleLimit(int servo_id, uint16_t cw_angle);
    bool setCCWAngleLimit(int servo_id, uint16_t ccw_angle);
    
    bool setVoltageLimits(int servo_id, float min_voltage_limit, float max_voltage_limit);
    bool setMinVoltageLimit(int servo_id, float min_voltage_limit);
    bool setMaxVoltageLimit(int servo_id, float max_voltage_limit);
    
    bool setTemperatureLimit(int servo_id, uint8_t max_temperature);
    bool setMaxTorque(int servo_id, uint16_t max_torque);
    bool setAlarmLed(int servo_id, uint8_t alarm_led);
    bool setAlarmShutdown(int servo_id, uint8_t alarm_shutdown);
    bool setTorqueEnable(int servo_id, bool on);
    bool setLed(int servo_id, bool on);
    
    bool setComplianceMargins(int servo_id, uint8_t cw_margin, uint8_t ccw_margin);
    bool setCWComplianceMargin(int servo_id, uint8_t cw_margin);
    bool setCCWComplianceMargin(int servo_id, uint8_t ccw_margin);

    bool setComplianceSlopes(int servo_id, uint8_t cw_slope, uint8_t ccw_slope);
    bool setCWComplianceSlope(int servo_id, uint8_t cw_slope);
    bool setCCWComplianceSlope(int servo_id, uint8_t ccw_slope);

    bool setPosition(int servo_id, uint16_t position);
    bool setVelocity(int servo_id, int16_t velocity);
    bool setTorqueLimit(int servo_id, uint16_t torque_limit);
    
    // ************************* SYNC_WRITE METHODS *************************** //
    bool setMultiPosition(std::vector<std::vector<int> > value_pairs);
    bool setMultiVelocity(std::vector<std::vector<int> > value_pairs);
    bool setMultiPositionVelocity(std::vector<std::vector<int> > value_pairs);
    bool setMultiComplianceMargins(std::vector<std::vector<int> > value_pairs);
    bool setMultiComplianceSlopes(std::vector<std::vector<int> > value_pairs);
    bool setMultiTorqueEnabled(std::vector<std::vector<int> > value_pairs);
    bool setMultiTorqueLimit(std::vector<std::vector<int> > value_pairs);
    bool setMultiValues(std::vector<std::map<std::string, int> > value_maps);
    
protected:
    std::map<int, DynamixelData*> cache_;
    std::set<int> connected_motors_;

    inline DynamixelData* findCachedParameters(int servo_id)
    {
        // this will either return an existing cache for servo_id or create new empty cahce and return that
        return cache_.insert(std::make_pair<int, DynamixelData*>(servo_id, new DynamixelData())).first->second;
    }
    
    bool updateCachedParameters(int servo_id, DynamixelData* data);
    void checkForErrors(int servo_id, uint8_t error_code, std::string command_failed);

    bool read(int servo_id,
              int address,
              int size,
              std::vector<uint8_t>& response);

    bool write(int servo_id,
               int address,
               const std::vector<uint8_t>& data,
               std::vector<uint8_t>& response);

    bool syncWrite(int address,
                   const std::vector<std::vector<uint8_t> >& data);
    
private:
    flexiport::Port* port_;
    pthread_mutex_t serial_mutex_;
    
    bool waitForBytes(ssize_t n_bytes, uint16_t timeout_ms);
    
    bool writePacket(const void* const buffer, size_t count);
    bool readResponse(std::vector<uint8_t>& response);
};

}

#endif
