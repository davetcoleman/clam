/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBX_NOVATEL_MESSAGES_H
#define GBX_NOVATEL_MESSAGES_H

#include <stdint.h> // for uint8_t and friends

namespace gbxnovatelutilacfr{
enum BinaryLogType{
    RawImuSBLogType = 325,
    BestGpsPosBLogType = 423,
    BestGpsVelBLogType = 506,
    InsPvaSBLogType = 508,
    InvalidLogType
};

#pragma pack(push,1)
// binary packet header; long version, mostly used for GPS gear
// according to:
// OEM4 Family Firmware Version 2.300 Command and Log Reference Rev 16; Table 4; Page 17
typedef struct Oem4BinaryHeader{        //offs, size,   comment
    uint8_t  sb1;                       //0     1       0xaa
    uint8_t  sb2;                       //1     1       0x44
    uint8_t  sb3;                       //2     1       0x12/0x13 , depending on long/short format
    uint8_t  headerLength;              //3     1
    uint16_t msgId;                     //4     2       which log we have
    uint8_t  msgType;                   //6     1       binary, ascii ...
    uint8_t  portAddress;               //7     1
    uint16_t msgLength;                 //8     2       _without_ header or CRC
    uint16_t seqNr;                     //10    2       for multiple related messages at a time (i.e. one per satellite)
    uint8_t  idleTime;                  //12    1
    uint8_t  timeStatus;                //13    1
    uint16_t gpsWeekNr;                 //14    2
    uint32_t msIntoWeek;                //16    4       milliseconds from beginning of week
    uint32_t receiverStatus;            //20    4
    uint16_t reserved;                  //24    2
    uint16_t swVersion;                 //26    2
}Oem4BinaryHeader;

// binary packet header; short version, mostly used for (higher rate) INS gear
// according to:
// SPAN Technology for OEMV User Manual Rev 3; Table 17; Page 74
typedef struct Oem4ShortBinaryHeader{   //offs, size,   comment
    uint8_t  sb1;                       //0     1       0xaa
    uint8_t  sb2;                       //1     1       0x44
    uint8_t  sb3;                       //2     1       0x12
    uint8_t  msgLength;                 //3     1       _without_ header or CRC
    uint16_t msgId;                     //4     2       which log we have
    uint16_t GpsWeekNr;                 //6     2
    uint32_t msIntoWeek;                //8     4       milliseconds from beginning of week
}Oem4ShortBinaryHeader;


typedef struct{                 //offs  size    comment
    uint32_t solutionStatus;    //0     4
    uint32_t positionType;      //4     4
    double   latitude;          //8     8       [deg] north positive
    double   longitude;         //16    8       [deg] east positive
    double   heightAMSL;        //24    8       [m] AMSL == above mean sea level (geoid)
    float    undulation;        //32    4       [m] aka geoidal seperation: undulation == heigth_ellipsoid - height_geoid/AMSL
    uint32_t datumId;           //36    4
    float    sigmaLatitude;     //40    4       [m] 1 standard deviation error estimate
    float    sigmaLongitude;    //44    4       [m] 1 standard deviation error estimate
    float    sigmaHeight;       //48    4       [m] 1 standard deviation error estimate
    int8_t   baseStationId[4];  //52    4
    float    diffAge;           //56    4       [s]
    float    solutionAge;       //60    4       [s]
    uint8_t  numObservations;   //64    1       number of observations tracked (?) L1 code/carrier/doppler + L2 code/carrier/doppler?
    uint8_t  numL1Ranges;       //65    1       number of L1 ranges used in computation (?)
    uint8_t  numL1RangesRTK;    //66    1       number of L1 ranges above the RTK mask angle (??) number of L1 carrier ranges used?
    uint8_t  numL2RangesRTK;    //67    1       number of L2 ranges above the RTK mask angle (??) number of L2 carrier ranges used?
    uint8_t  reserved0;         //68    1
    uint8_t  reserved1;         //69    1
    uint8_t  reserved2;         //70    1
    uint8_t  reserved3;         //71    1
    uint32_t crc;               //72    4
}BestGpsPosData;

typedef struct{
    Oem4BinaryHeader header;
    BestGpsPosData   data;
}BestGpsPosLogB;

typedef struct{                 //offs  size    comment
    uint32_t solutionStatus;    //0     4
    uint32_t velocityType;      //4     4
    float    latency;           //8     4       [s]
    float    diffAge;           //12    4       [s]
    double   horizontalSpeed;   //16    8       [m/s]
    double   trackOverGround;   //24    8       [deg] w respect to true North
    double   verticalSpeed;     //32    8       [m/s]
    float    reserved;          //40    4
    uint32_t crc;               //44    4
}BestGpsVelData;

typedef struct{
    Oem4BinaryHeader header;
    BestGpsVelData   data;
}BestGpsVelLogB;

typedef struct{                 //offs  size    comment
    uint32_t gpsWeekNr;         //0     4
    double   secIntoWeek;       //4     8
    double   latitude;          //12    8       [deg] north positive WGS84
    double   longitude;         //20    8       [deg] east positive WGS84
    double   height;            //28    8       [m] above ellipsoid WGS84 (heigth_ellipsoid - undulation == height_geoid/AMSL)
    double   northVelocity;     //36    8       [m/s] south is negative; true north?
    double   eastVelocity;      //44    8       [m/s] west is negative; true east?
    double   upVelocity;        //52    8       [m/s] down is negative; geoid/ellipsoid vertical?
    //The default IMU axis definitions are:
    //  Y - forward
    //  Z - up
    //  X - right hand side
    double   roll;              //60    8       [degree] right handed rotation from local level around y-axes
    double   pitch;             //68    8       [degree] right handed rotation from local level around x-axes
    double   azimuth;           //60    8       [degree] left handed around z-axes rotation from (true?) north clockwise
    uint32_t insStatus;         //68    4
    uint32_t crc;               //72    4
}InsPvaData;

//binary log with full headers
typedef struct{
    Oem4BinaryHeader header;
    InsPvaData   data;
}InsPvaLogB;

//binary log with short header; *preferred*, since this log is usually done at high rate
typedef struct{
    Oem4ShortBinaryHeader header;
    InsPvaData   data;
}InsPvaLogSB;

typedef struct{                 //offs  size    comment
    uint32_t gpsWeekNr;         //0     4
    double   secIntoWeek;       //4     8
    uint32_t imuStatus;         //12    4
    //The default IMU axis definitions are:
    //  Y - forward
    //  Z - up
    //  X - out the right hand side
    int32_t zAccelCnt;          //16    4
    int32_t yNegativAccelCnt ;  //20    4
    int32_t xAccelCnt;          //24    4
    int32_t zGyroCnt;           //28    4
    int32_t yNegativGyroCnt;    //32    4
    int32_t xGyroCnt;           //36    4
    int32_t crc;                //40    4
}RawImuData;

//binary log with full headers
typedef struct{
    Oem4BinaryHeader header;
    RawImuData   data;
}RawImuLogB;

//binary log with short header; *preferred*, since this log is done at high rate
typedef struct{
    Oem4ShortBinaryHeader header;
    RawImuData   data;
}RawImuLogSB;
#pragma pack(pop)
} //namespace
#endif
