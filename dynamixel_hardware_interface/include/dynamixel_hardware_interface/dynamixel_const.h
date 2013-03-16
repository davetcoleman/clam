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

#ifndef DYNAMIXEL_CONST_H__
#define DYNAMIXEL_CONST_H__

#include <stdint.h>
#include <string>

namespace dynamixel_hardware_interface
{

const uint16_t DXL_MAX_LOAD_ENCODER = 1023;     // 0x3FF
const uint16_t DXL_MAX_VELOCITY_ENCODER = 1023; // 0x3FF
const uint16_t DXL_MAX_TORQUE_ENCODER = 1023;   // 0x3FF

// Control Table Constants
typedef enum DynamixelControlEnum
{
    DXL_MODEL_NUMBER_L = 0,
    DXL_MODEL_NUMBER_H = 1,
    DXL_FIRMWARE_VERSION = 2,
    DXL_ID = 3,
    DXL_BAUD_RATE = 4,
    DXL_RETURN_DELAY_TIME = 5,
    DXL_CW_ANGLE_LIMIT_L = 6,
    DXL_CW_ANGLE_LIMIT_H = 7,
    DXL_CCW_ANGLE_LIMIT_L = 8,
    DXL_CCW_ANGLE_LIMIT_H = 9,
    DXL_DRIVE_MODE = 10,
    DXL_LIMIT_TEMPERATURE = 11,
    DXL_DOWN_LIMIT_VOLTAGE = 12,
    DXL_UP_LIMIT_VOLTAGE = 13,
    DXL_MAX_TORQUE_L = 14,
    DXL_MAX_TORQUE_H = 15,
    DXL_RETURN_LEVEL = 16,
    DXL_ALARM_LED = 17,
    DXL_ALARM_SHUTDOWN = 18,
    DXL_OPERATING_MODE = 19,
    DXL_DOWN_CALIBRATION_L = 20,
    DXL_DOWN_CALIBRATION_H = 21,
    DXL_UP_CALIBRATION_L = 22,
    DXL_UP_CALIBRATION_H = 23,
    DXL_TORQUE_ENABLE = 24,
    DXL_LED = 25,
    DXL_CW_COMPLIANCE_MARGIN = 26,
    DXL_CCW_COMPLIANCE_MARGIN = 27,
    DXL_CW_COMPLIANCE_SLOPE = 28,
    DXL_CCW_COMPLIANCE_SLOPE = 29,
    DXL_GOAL_POSITION_L = 30,
    DXL_GOAL_POSITION_H = 31,
    DXL_GOAL_SPEED_L = 32,
    DXL_GOAL_SPEED_H = 33,
    DXL_TORQUE_LIMIT_L = 34,
    DXL_TORQUE_LIMIT_H = 35,
    DXL_PRESENT_POSITION_L = 36,
    DXL_PRESENT_POSITION_H = 37,
    DXL_PRESENT_SPEED_L = 38,
    DXL_PRESENT_SPEED_H = 39,
    DXL_PRESENT_LOAD_L = 40,
    DXL_PRESENT_LOAD_H = 41,
    DXL_PRESENT_VOLTAGE = 42,
    DXL_PRESENT_TEMPERATURE = 43,
    DXL_REGISTERED_INSTRUCTION = 44,
    DXL_PAUSE_TIME = 45,
    DXL_MOVING = 46,
    DXL_LOCK = 47,
    DXL_PUNCH_L = 48,
    DXL_PUNCH_H = 49,
    DXL_SENSED_CURRENT_L = 56,
    DXL_SENSED_CURRENT_H = 57,

} DynamixelControl;

typedef enum DynamixelInstructionEnum
{
    DXL_PING = 1,
    DXL_READ_DATA = 2,
    DXL_WRITE_DATA = 3,
    DXL_REG_WRITE = 4,
    DXL_ACTION = 5,
    DXL_RESET = 6,
    DXL_SYNC_WRITE = 131,
    DXL_BROADCAST = 254,

} DynamixelInstruction;

typedef enum DynamixelErrorCodeEnum
{
    DXL_INSTRUCTION_ERROR = 64,
    DXL_OVERLOAD_ERROR = 32,
    DXL_CHECKSUM_ERROR = 16,
    DXL_RANGE_ERROR = 8,
    DXL_OVERHEATING_ERROR = 4,
    DXL_ANGLE_LIMIT_ERROR = 2,
    DXL_INPUT_VOLTAGE_ERROR = 1,
    DXL_NO_ERROR = 0,

} DynamixelErrorCode;


typedef enum DynamixelParamsEnum
{
    ENCODER_RESOLUTION,
    RANGE_DEGREES,
    TORQUE_PER_VOLT,
    VELOCITY_PER_VOLT

} DynamixelParams;

inline std::string getMotorModelName(int model_number)
{
    if (model_number == 113) { return "DX-113"; }
    else if (model_number == 116) { return "DX-116"; }
    else if (model_number == 117) { return "DX-117"; }
    else if (model_number == 12) { return "AX-12"; }
    else if (model_number == 18) { return "AX-18"; }
    else if (model_number == 10) { return "RX-10"; }
    else if (model_number == 24) { return "RX-24"; }
    else if (model_number == 28) { return "RX-28"; }
    else if (model_number == 64) { return "RX-64"; }
    else if (model_number == 107) { return "EX-106"; }
    else if (model_number == 29) { return "MX-28"; }

    return "";
}

const double KGCM_TO_NM = 0.0980665;        // 1 kg-cm is that many N-m
const double RPM_TO_RADSEC = 0.104719755;   // 1 RPM is that many rad/sec

inline double getMotorModelParams(int model_number, DynamixelParams param)
{
    if (model_number == 113)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (10.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (54.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 116)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (21.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (78.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 117)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (37.0 * KGCM_TO_NM) / 18.5; }
            case VELOCITY_PER_VOLT:  { return (85.0 * RPM_TO_RADSEC) / 18.5; }
        }
    }
    else if (model_number == 12)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (15.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (59.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 18)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (18.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (97.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 10)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (13.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (54.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 24)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (26.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (126.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }
    else if (model_number == 28)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (37.0 * KGCM_TO_NM) / 18.5; }
            case VELOCITY_PER_VOLT:  { return (85.0 * RPM_TO_RADSEC) / 18.5; }
        }
    }
    else if (model_number == 64)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 1024.0; }
            case RANGE_DEGREES:      { return 300.0; }
            case TORQUE_PER_VOLT:    { return (52.0 * KGCM_TO_NM) / 18.5; }
            case VELOCITY_PER_VOLT:  { return (64.0 * RPM_TO_RADSEC) / 18.5; }
        }
    }
    else if (model_number == 107)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 4096.0; }
            case RANGE_DEGREES:      { return 250.92; }
            case TORQUE_PER_VOLT:    { return (107.0 * KGCM_TO_NM) / 18.5; }
            case VELOCITY_PER_VOLT:  { return (91.0 * RPM_TO_RADSEC) / 18.5; }
        }
    }
    else if (model_number == 29)
    {
        switch (param)
        {
            case ENCODER_RESOLUTION: { return 4096.0; }
            case RANGE_DEGREES:      { return 360.0; }
            case TORQUE_PER_VOLT:    { return (24.0 * KGCM_TO_NM) / 12.0; }
            case VELOCITY_PER_VOLT:  { return (54.0 * RPM_TO_RADSEC) / 12.0; }
        }
    }

    return -1;
}

}

#endif  // DYNAMIXEL_CONST_H__
