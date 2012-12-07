/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBX_NOVATEL_IMUDECODER_H
#define GBX_NOVATEL_IMUDECODER_H

#include <stdint.h>
#include <string>
namespace gbxnovatelutilacfr{
//the novatel span system supports different IMUs, with different status bit-fields and different conversion factors

//yield [rad]
const double GyroConstantHg1700Ag11Ag58 = 1.16415321826934814453e-10;
const double GyroConstantHg1700Ag17Ag62 = 1.16415321826934814453e-10; //verified on Hw
const double GyroConstantImarFsas = 4.8481368110953599359e-7;
const double GyroConstantLn200 = 1.9073486328125e-6;

//yield [m/s]
const double AccelConstantHg1700Ag11Ag58 = 2.27094150781631469727e-9;
const double AccelConstantHg1700Ag17Ag62 = 4.54188301563262939453e-9; //verified on Hw
const double AccelConstantImarFsas = 1.52587890625e-6;
const double AccelConstantLn200 = 6.103515625e-5;

class ImuDecoder {
public:
    virtual ~ImuDecoder(){}
    inline double gyroCnt2Rad(const int32_t cnt){ return cnt*gyroConstant_; }
    inline double accelCnt2MperSec(const int32_t cnt){ return cnt*accelConstant_; }
    virtual inline bool statusIsGood(const uint32_t imuStatus){return true;} // possibly we need isWarning(), isError() as well. Currently anything not good is treated as error.
    virtual std::string statusToString(const uint32_t imuStatus)=0;

protected:
    double gyroConstant_;
    double accelConstant_;
};

extern "C"{
    ImuDecoder *createImuDecoder(std::string type);
}

}//namespace

#endif
