/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include "imudecoder.h"
#include <gbxutilacfr/gbxutilacfr.h>
#include <sstream>
#include <iostream>
namespace gnua = gbxnovatelutilacfr;
namespace{
//HG1700 family has the same status decoding, but different accelerometer constant
class ImuDecoderHg1700 : public gnua::ImuDecoder {
public:
    virtual ~ImuDecoderHg1700(){}
    inline bool statusIsGood(const uint32_t imuStatus){
        return 0 == ((imuStatus & 0xf8000010));
    }
    std::string statusToString(const uint32_t imuStatus);
};

class ImuDecoderHg1700Ag11Ag58 : public ImuDecoderHg1700{
public:
    ImuDecoderHg1700Ag11Ag58(){
        gyroConstant_ = gnua::GyroConstantHg1700Ag11Ag58;
        accelConstant_ = gnua::AccelConstantHg1700Ag11Ag58;
    };
};
class ImuDecoderHg1700Ag17Ag62 : public ImuDecoderHg1700{
public:
    ImuDecoderHg1700Ag17Ag62(){
        gyroConstant_ = gnua::GyroConstantHg1700Ag17Ag62;
        accelConstant_ = gnua::AccelConstantHg1700Ag17Ag62;
    };
};

std::string
ImuDecoderHg1700::statusToString(const uint32_t imuStatus){
    std::stringstream ss;
    ss << "IMU test: "
        << (( 0 == (imuStatus & 0x00000010)) ? "pass;" : "fail;") << " ";
    ss << "Z-gyro path-length control: "
        << (( 0 == (imuStatus & 0x00000020)) ? "good;" : "fail;") << " ";
    ss << "Y-gyro path-length control: "
        << (( 0 == (imuStatus & 0x00000040)) ? "good;" : "fail;") << " ";
    ss << "X-gyro path-length control: "
        << (( 0 == (imuStatus & 0x00000080)) ? "good;" : "fail;") << " ";
    ss << "Gyro tests: "
        << (( 0 == (imuStatus & 0x08000000)) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer tests: "
        << (( 0 == (imuStatus & 0x10000000)) ? "pass;" : "fail;") << " ";
    ss << "Other test: "
        << (( 0 == (imuStatus & 0x20000000)) ? "pass;" : "fail;") << " ";
    ss << "Memory test: "
        << (( 0 == (imuStatus & 0x40000000)) ? "pass;" : "fail;") << " ";
    ss << "Processor test: "
        << (( 0 == (imuStatus & 0x80000000)) ? "pass;" : "fail;") << " ";
    int temprAccel = ((imuStatus & 0x0000ff00)) >> 8;
    int softwVersion = ((imuStatus & 0x00ff0000)) >> 16;
    ss << "Accelerometer temprature [C]: " << temprAccel << " ";
    ss << "Software version number: " << softwVersion;

    return ss.str();
}

class ImuDecoderImarFsas : public gnua::ImuDecoder{
public:
    ImuDecoderImarFsas(){
        gyroConstant_ = gnua::GyroConstantImarFsas;
        accelConstant_ = gnua::AccelConstantImarFsas;
        std::cerr << "This driver has _not_ been tested with this IMU!\n";
    };
    inline bool statusIsGood(const uint32_t imuStatus){
        return 0 == ((imuStatus & 0xefec9580));
    }
    std::string statusToString(const uint32_t imuStatus);
};

std::string
ImuDecoderImarFsas::statusToString(const uint32_t imuStatus){
    std::stringstream ss;
    ss << "Gyro warm-up: "
        << (( 0 == (imuStatus & 0x00000010 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro self-test active: "
        << (( 0 == (imuStatus & 0x00000020 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro status bit set: "
        << (( 0 == (imuStatus & 0x00000040 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro time-out command interface: "
        << (( 0 == (imuStatus & 0x00000080 )) ? "pass;" : "fail;") << " ";
    ss << "Power-up built-in test (PBIT): "
        << (( 0 == (imuStatus & 0x00000100 )) ? "pass;" : "fail;") << " ";
    ss << "Interrupt: "
        << (( 0 == (imuStatus & 0x00000400 )) ? "pass;" : "fail;") << " ";
    ss << "Warm-up: "
        << (( 0 == (imuStatus & 0x00001000 )) ? "pass;" : "fail;") << " ";
    ss << "Initiated built-in test (IBIT): "
        << (( 0 == (imuStatus & 0x00008000 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer: "
        << (( 0 == (imuStatus & 0x00040000 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer time-out: "
        << (( 0 == (imuStatus & 0x00080000 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro initiated BIT: "
        << (( 0 == (imuStatus & 0x00200000 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro self-test: "
        << (( 0 == (imuStatus & 0x00400000 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro time-out: "
        << (( 0 == (imuStatus & 0x00800000 )) ? "pass;" : "fail;") << " ";
    ss << "Analog-to-Digital (AD): "
        << (( 0 == (imuStatus & 0x01000000 )) ? "pass;" : "fail;") << " ";
    ss << "Testmode: "
        << (( 0 == (imuStatus & 0x02000000 )) ? "pass;" : "fail;") << " ";
    ss << "Software: "
        << (( 0 == (imuStatus & 0x04000000 )) ? "pass;" : "fail;") << " ";
    ss << "RAM/ROM: "
        << (( 0 == (imuStatus & 0x08000000 )) ? "pass;" : "fail;") << " ";
    ss << "Operational: "
        << (( 0 == (imuStatus & 0x20000000 )) ? "pass;" : "fail;") << " ";
    ss << "Interface: "
        << (( 0 == (imuStatus & 0x40000000 )) ? "pass;" : "fail;") << " ";
    ss << "Interface time-out: "
        << (( 0 == (imuStatus & 0x80000000 )) ? "pass;" : "fail;") << " ";

    return ss.str();
}

class ImuDecoderLn200 : public gnua::ImuDecoder{
public:
    ~ImuDecoderLn200(){}
    ImuDecoderLn200(){
        gyroConstant_ = gnua::GyroConstantLn200;
        accelConstant_ = gnua::AccelConstantLn200;
        std::cerr << "This driver has _not_ been tested with this IMU!\n";
    };
    inline bool statusIsGood(const uint32_t imuStatus){
        return 0 == ((imuStatus & 0xf8000010));
    }
    std::string statusToString(const uint32_t imuStatus);
};

std::string
ImuDecoderLn200::statusToString(const uint32_t imuStatus){
    std::stringstream ss;

    ss << "Delta_velocity_counter: "
        << (( 0 == (imuStatus & 0x00000001 )) ? "pass;" : "fail;") << " ";
    ss << "D/A_converter: "
        << (( 0 == (imuStatus & 0x00000002 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro: "
        << (( 0 == (imuStatus & 0x00000004 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer: "
        << (( 0 == (imuStatus & 0x00000008 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro_loop_control: "
        << (( 0 == (imuStatus & 0x00000010 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro_temperature_control: "
        << (( 0 == (imuStatus & 0x00000020 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer_temperature: "
        << (( 0 == (imuStatus & 0x00000040 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer_temperature: "
        << (( 0 == (imuStatus & 0x00000040 )) ? "pass;" : "fail;") << " ";
    ss << "A/D_converter: "
        << (( 0 == (imuStatus & 0x00000100 )) ? "pass;" : "fail;") << " ";
    ss << "Serial_I/O: "
        << (( 0 == (imuStatus & 0x00000200 )) ? "pass;" : "fail;") << " ";
    ss << "Serial_I/O: "
        << (( 0 == (imuStatus & 0x00000200 )) ? "pass;" : "fail;") << " ";
    ss << "Laser_diode: "
        << (( 0 == (imuStatus & 0x00000800 )) ? "pass;" : "fail;") << " ";
    ss << "Thermo-electric_cooler_(TEC): "
        << (( 0 == (imuStatus & 0x00001000 )) ? "pass;" : "fail;") << " ";
    ss << "Broadband_Fiber_Source_(BFS)_fiber_temperature: "
        << (( 0 == (imuStatus & 0x00002000 )) ? "pass;" : "fail;") << " ";
    ss << "Optical_receiver: "
        << (( 0 == (imuStatus & 0x00004000 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro_accuracy: "
        << (( 0 == (imuStatus & 0x01000000 )) ? "pass;" : "fail;") << " ";
    ss << "Gyro: "
        << (( 0 == (imuStatus & 0x02000000 )) ? "pass;" : "fail;") << " ";
    ss << "Shut_down_on_failure: "
        << (( 0 == (imuStatus & 0x04000000 )) ? "pass;" : "fail;") << " ";
    ss << "Fast_start: "
        << (( 0 == (imuStatus & 0x08000000 )) ? "pass;" : "fail;") << " ";
    ss << "Commanded_bit_in_progress: "
        << (( 0 == (imuStatus & 0x10000000 )) ? "pass;" : "fail;") << " ";
    ss << "Accelerometer_data: "
        << (( 0 == (imuStatus & 0x40000000 )) ? "pass;" : "fail;") << " ";

    return ss.str();
}
}

namespace gbxnovatelutilacfr{
ImuDecoder*
createImuDecoder(std::string type){
    if (0 == type.compare("IMU_HG1700_AG11")
            || 0 == type.compare("IMU_HG1700_AG58")){
        return new ImuDecoderHg1700Ag11Ag58;
    }
    if (0 == type.compare("IMU_HG1700_AG17")
            || 0 == type.compare("IMU_HG1700_AG62")){
        return new ImuDecoderHg1700Ag17Ag62;
    }
    if ( 0 == type.compare("IMU_LN200")
            || 0 == type.compare("IMU_LN200_400HZ")){
        return new ImuDecoderLn200;
    }
    if ( 0 == type.compare("IMU_IMAR_FSAS")){
        return new ImuDecoderImarFsas;
    }
    //oops
    throw gbxutilacfr::Exception( ERROR_INFO, "Unknown Imu type!");
}
}//namespace


namespace{
}//namespace
