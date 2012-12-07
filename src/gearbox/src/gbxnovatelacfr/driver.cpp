/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Matthew Ridley, Ben Upcroft, Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <gbxnovatelacfr/driver.h>

#include <gbxnovatelacfr/gbxnovatelutilacfr/serialconnectivity.h>
#include <gbxnovatelacfr/gbxnovatelutilacfr/novatelmessages.h>
#include <gbxnovatelacfr/gbxnovatelutilacfr/imudecoder.h>
#include <gbxnovatelacfr/gbxnovatelutilacfr/receiverstatusdecoder.h>
#include <gbxnovatelacfr/gbxnovatelutilacfr/crc32.h>

#include <gbxserialacfr/gbxserialacfr.h>
#include <gbxutilacfr/gbxutilacfr.h>
#include <gbxutilacfr/trivialtracer.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>

#include <sys/time.h>
#include <time.h>

using namespace std;
using namespace gbxserialacfr;

namespace gnua = gbxnovatelutilacfr;
namespace gna = gbxnovatelacfr;
namespace gua = gbxutilacfr;

namespace {
#pragma pack(push,1)
    // Our quick and dirty message-decoder
    // Maximum size is (legacy) hardcoded to 516bytes. The real upper limit (for long headers) would be 65535+28+4 bytes.
    // Problems should hopefully be picked up by checkParserAssumptions().
    //
    // Todo: write a proper parser
    static const int rawMsgSize = 516; // maximum size of message we can decode (including header, crc, and data)
    static const int longMsgDataSize = rawMsgSize - sizeof(gnua::Oem4BinaryHeader);
    static const int shortMsgDataSize = rawMsgSize - sizeof(gnua::Oem4ShortBinaryHeader);
    union NovatelMessage{
        struct{
            gnua::Oem4BinaryHeader header;
            uint8_t               data[longMsgDataSize];
        };
        struct{
            gnua::Oem4ShortBinaryHeader shortHeader;
            uint8_t               shortData[shortMsgDataSize];
        };
        uint8_t rawMessage[rawMsgSize];

        // these guys are used to directly decode messages;
        // obviously fails on endian mismatch, any sort of size mismatch and is rather nasty in general;
        // feel free to implement something better
        gnua::BestGpsPosLogB bestGpsPos;
        gnua::BestGpsVelLogB bestGpsVel;
        gnua::InsPvaLogSB   insPva;
        gnua::RawImuLogSB   rawImu;
    };
#pragma pack(pop)
    // this guy checks if the assumptions for the gear above are correct (abort()s through assert() otherwise)
    void checkParserAssumptions();

    // returns the id of the message it read (msg), or throws an gbxutilacfr::Exception
    uint16_t readNovatelMessage(union NovatelMessage &msg, struct timeval &timeStamp, gbxserialacfr::Serial *serial);

    // take novatel data and create stuff according to our external api
    std::auto_ptr<gna::GenericData> createExternalMsg(gnua::InsPvaLogSB &insPva, struct timeval &timeStamp);
    std::auto_ptr<gna::GenericData> createExternalMsg(gnua::BestGpsPosLogB &bestGpsPos, struct timeval &timeStamp);
    std::auto_ptr<gna::GenericData> createExternalMsg(gnua::BestGpsVelLogB &bestGpsVel, struct timeval &timeStamp);
    std::auto_ptr<gna::GenericData> createExternalMsg(gnua::RawImuLogSB &rawImu, struct timeval &timeStamp, gnua::ImuDecoder *imuDecoder);
    enum gna::GpsSolutionStatusType externalGpsSolutionStatus(uint32_t novatelGpsSolutionStatus);
    enum gna::GpsPosVelType externalGpsPosVelType(uint32_t novatelGpsPosVelType);

    // helper functions for the toString() gear
    std::string statusToString(gna::StatusMessageType statusMessageType, std::string statusMessage);
    std::string doubleVectorToString(const vector<double > &vec, const std::string seperator = std::string(" "));
}

namespace gbxnovatelacfr
{
Driver::Driver( const Config& cfg) :
    baud_(115200),
    config_(cfg),
    tracerInternal_(new gbxutilacfr::TrivialTracer()),
    tracer_(*(tracerInternal_.get()))
{
    //assert(0 != tracer_.get());
    configure();
    return;
}

Driver::Driver( const Config& cfg,
        gbxutilacfr::Tracer &tracer) :
    baud_(115200),
    config_(cfg),
    tracerInternal_(0),
    tracer_(tracer)
{
    //assert(0 != tracer_.get());
    configure();
    return;
}

void
Driver::configure( ) {
    if(false == config_.isValid()){
        throw (gua::Exception(ERROR_INFO, "Invalid Configuration!"));
    }

    checkParserAssumptions();

    // configure serial port
    baud_ = config_.baudRate_;
    std::string serialDevice = config_.serialDevice_;
    serial_.reset(new Serial( serialDevice, baud_, Serial::Timeout(1,0) ));
    assert(0 != serial_.get());
    serial_->setDebugLevel(0);

    connectToHardware();
    // just in case something is running... stops the novatel logging any messages
    serial_->writeString( "unlogall\r\n" );
    serial_->drain();
    serial_->flush();
    configureImu();
    configureIns();
    configureGps();
    requestData();
    serial_->flush();
    tracer_.info("Setup done, starting normal operation!");
    return;
}

Driver::~Driver() {
    // just in case something is running... stops the novatel logging any messages
    try{
        tracer_.info("Stopping NovatelSpan driver!");
        serial_->flush();
        serial_->writeString( "unlogall\r\n" );
        serial_->drain();
        tracer_.info("NovatelSpan driver stopped!");
    }
    catch(...){
        //no throwing from destructors
    }
}

void
Driver::connectToHardware() {
    // baudrates we test for; this is
    // _not_ all the baudrates the receiver
    // can possible be set to
    int baudrates[]={
        9600,
        19200,
        38400,
        57600,
        115200,
        230400
    };
    int currentBaudrate = 0;
    bool correctBaudrate = false;

    tracer_.info( "Trying to hook up to receiver at different Baudrates" );
    int maxTry = 4;
    int successThresh = 4;
    int timeOutMsec = 250;
    std::string challenge("unlogall\r\n");
    std::string ack("<OK");
    size_t i=0;
    while(false == correctBaudrate && i<sizeof baudrates/sizeof baudrates[0]){
        currentBaudrate = baudrates[i];
        correctBaudrate = gnua::testConnectivity( challenge, ack, *(serial_.get()), timeOutMsec, maxTry, successThresh, currentBaudrate);
        i++;
    }
    if(false == correctBaudrate){
        throw ( gua::Exception(ERROR_INFO, "!Failed to establish a connection to the receiver! Check physical connections; Check manually (minicom) for Baudrates < 9600kb/s."));
    }
    // ok, we've got a working link
    std::stringstream ss;
    ss << "Established connection at "
        << currentBaudrate << "bps; "
        << "Resetting to configured speed: "
        << baud_ << "bps";
    tracer_.info(ss.str());
    char str[256];
    sprintf( str,"com com1 %d n 8 1 n off on\r\n", baud_ );
    serial_->writeString( str );
    if(false == gnua::testConnectivity( challenge, ack, *(serial_.get()), timeOutMsec, maxTry, successThresh, baud_)){
        throw ( gua::Exception(ERROR_INFO, "!Failed to reset connection to configured baudrate!"));
    }
    return;
}

void
Driver::configureImu() {
    std::string challenge = "";
    std::string ack = "<OK";
    std::string errorResponse = "";
    int timeOutMsec = 200;

    if(config_.enableImu_){
        tracer_.info("Configuring IMU, switching INS ON!");
        imuDecoder_.reset(gnua::createImuDecoder(config_.imuType_));
        std::stringstream ss;
        // tell the novatel what serial port the imu is attached to (com3 == aux)
        challenge = ( "interfacemode com3 imu imu on\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
        // the type of imu being used
        ss << "setimutype "
            << config_.imuType_
            << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
        challenge = "inscommand enable\r\n";
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
        //force the IMU to re-align at every startup
        //put = serial_->writeString( "inscommand reset\r\n" );
        //tracer_.info("Reset IMU; Waiting 5 seconds before continuing!");
        //sleep(5);
    }else{
        tracer_.info("No IMU, switching INS OFF!");
        challenge = "inscommand disable\r\n";
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }
    return;
}

void
Driver::configureIns() {
    std::string challenge = "";
    std::string ack = "<OK";
    std::string errorResponse = "";
    int timeOutMsec = 200;

    if(config_.enableSetImuOrientation_ && config_.enableImu_){
        std::stringstream ss;
        // imu orientation constant
        // this tells the imu where its z axis (up) is pointing. constants defined in manual.
        // with imu mounted upside down, constant is 6 and axes are remapped: x = y, y = x, -z = z 
        ss << "setimuorientation " << config_.setImuOrientation_ << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    if(config_.enableVehicleBodyRotation_ && config_.enableImu_){
        std::stringstream ss;
        // vehicle to imu body rotation
        // angular offset from the vehicle to the imu body. unclear how this relates to imu orientation command 
        // the novatel docs are not especially clear on this stuff; It's highly recommended to mount the IMU
        // exactly as advised by novatel and just ignore this
        ss << "vehiclebodyrotation "
            << config_.vehicleBodyRotation_[0]
            << config_.vehicleBodyRotation_[1]
            << config_.vehicleBodyRotation_[2];
        if(3 == config_.vehicleBodyRotationUncertainty_.size()){
            // optional, vehicle to imu body rotation uncertainty
            ss << config_.vehicleBodyRotationUncertainty_[0]
                << config_.vehicleBodyRotationUncertainty_[1]
                << config_.vehicleBodyRotationUncertainty_[2];
        }
        ss << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    if(config_.enableImu_){
        std::stringstream ss;
        // The span system kalman fiter needs this info; make _sure_ you do this right
        ss << "setimutoantoffset "
            << config_.imuToGpsOffset_[0] << " "
            << config_.imuToGpsOffset_[1] << " "
            << config_.imuToGpsOffset_[2];

        if( 3 == config_.imuToGpsOffsetUncertainty_.size() ){
            ss << " "
                << config_.imuToGpsOffsetUncertainty_[0] << " "
                << config_.imuToGpsOffsetUncertainty_[1] << " "
                << config_.imuToGpsOffsetUncertainty_[2];
        }
        ss << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }
    return;
}

void
Driver::configureGps() {
    std::string challenge = "";
    std::string ack = "<OK";
    std::string errorResponse = "";
    int timeOutMsec = 200;

    // hardcoded settings first

    // turn off posave as this command implements position averaging for base stations.
    // make sure that fixposition has not been set
    challenge = ( "fix none\r\n" );
    if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
        std::stringstream ss;
        ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
        throw ( gua::Exception(ERROR_INFO, ss.str()));
    }
    // select the geodetic datum for operation of the receiver (wgs84 = default)
    challenge = ( "datum wgs84\r\n" );
    if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
        std::stringstream ss;
        ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
        throw ( gua::Exception(ERROR_INFO, ss.str()));
    }
    //Let the receiver figure out which range corrections are best
    challenge = ( "PSRDIFFSOURCE AUTO\r\n" );
    if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
        std::stringstream ss;
        ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
        throw ( gua::Exception(ERROR_INFO, ss.str()));
    }

    // CDGPS
    if(config_.enableCDGPS_){
        tracer_.info("Turning on CDGPS!");
        challenge = ( "ASSIGNLBAND CDGPS 1547547 4800\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    // turn SBAS on/off (essentially global DGPS)
    if(config_.enableSBAS_){
        tracer_.info("Turning on SBAS!");
        challenge = ( "SBASCONTROL ENABLE Auto 0 ZEROTOTWO\r\n");
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
        //we try to use WAAS satellites even below the horizon
        challenge = ( "WAASECUTOFF -5.0\r\n");
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }
    else{
        tracer_.info("Turning off SBAS!");
        challenge = ( "SBASCONTROL DISABLE Auto 0 NONE\r\n");
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    // rtk
    if(config_.enableRTK_){
        tracer_.info("Turning on RTK!");
        challenge = ( "com com2,9600,n,8,1,n,off,on\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
        challenge = ( "interfacemode com2 rtca none\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    if(config_.enableUseOfOmniStarCarrier_){
        //Let the receiver figure out which rtk corrections are best
        challenge = ( "RTKSOURCE AUTO\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }else{
        //We only use our own rtk corrections; _not_ OmniSTAR HP/XP
        challenge = ( "RTKSOURCE RTCA ANY\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }
    return;
}

void
Driver::requestData() {
    std::string challenge = "";
    std::string ack = "<OK";
    std::string errorResponse = "";
    int timeOutMsec = 200;
    //we assume that the config_ has been checked at this point (isValid())
    //so we don't need to check that the rates make sense

    // GPS messages

    // gps position without ins
    if(config_.enableGpsPos_){
        std::stringstream ss;
        ss << "Turning on GPS position at " << 1.0/config_.dtGpsPos_ << "Hz!";
        tracer_.info(ss.str().c_str());
        ss.str("");
        ss << "log bestgpsposb ontime " << config_.dtGpsPos_ << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    // gps velocity without ins
    if(config_.enableGpsVel_){
        std::stringstream ss;
        ss << "Turning on GPS velocity at " << 1.0/config_.dtGpsVel_ << "Hz!";
        tracer_.info(ss.str().c_str());
        ss.str("");
        ss << "log bestgpsvelb ontime " << config_.dtGpsVel_ << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }


    // INS messages

    // pva data in wgs84 coordinates
    if(config_.enableInsPva_){
        std::stringstream ss;
        ss << "Turning on INS position/velocity/orientation at " << 1.0/config_.dtInsPva_ << "Hz!";
        tracer_.info(ss.str().c_str());
        ss.str("");
        ss << "log inspvasb ontime " << config_.dtInsPva_ << "\r\n";
        challenge = ss.str();
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            ss.str("");
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }


    // IMU messages

    // raw accelerometer and gyro data
    if(config_.enableRawImu_){
        tracer_.info("Turning on raw imu data!");
        challenge = ( "log rawimusb onnew\r\n" );
        if(false == gnua::sendCmdWaitForResponse(challenge, ack, errorResponse, *(serial_.get()), timeOutMsec)){
            std::stringstream ss;
            ss << " Failure!\n Tried to send: " << challenge << " Receiver responded: " << errorResponse;
            throw ( gua::Exception(ERROR_INFO, ss.str()));
        }
    }

    return;
}

std::auto_ptr<GenericData>
Driver::read(){
    union NovatelMessage msg;
    std::auto_ptr<GenericData> data;
    data.reset(0);
    struct timeval timeStamp = {0,0};

    // read msg from hardware
    do{
        // Timeouts are not adjusted once a serial call returns;
        // So we could be stuck here for longer than the set timeout.
        int ret = serial_->bytesAvailableWait();
        if ( ret > 0 ) {
            switch(readNovatelMessage(msg, timeStamp, serial_.get())){
                case gnua::InsPvaSBLogType:
                    data = createExternalMsg(msg.insPva, timeStamp);
                    break;
                case gnua::BestGpsVelBLogType:
                    data = createExternalMsg(msg.bestGpsVel, timeStamp);
                    break;
                case gnua::BestGpsPosBLogType:
                    data = createExternalMsg(msg.bestGpsPos, timeStamp);
                    break;
                case gnua::RawImuSBLogType:
                    data = createExternalMsg(msg.rawImu, timeStamp, imuDecoder_.get());
                    break;
                case gnua::InvalidLogType:
                    {
                        std::stringstream ss;
                        ss <<"Id invalid, looks like we didn't get anything from the receiver!" << std::endl;
                        throw ( gua::Exception(ERROR_INFO, ss.str()) );
                    }
                    break;
                default:
                    {
                        std::stringstream ss;
                        ss <<"Got unexpected message type from receiver; id: " << msg.header.msgId << std::endl;
                        if(config_.ignoreUnknownMessages_){
                            tracer_.warning(ss.str());
                        }else{
                            throw ( gua::Exception(ERROR_INFO, ss.str()) );
                        }
                    }
                    break;
            }
        }
        else {
            throw ( gua::Exception(ERROR_INFO, "Timed out while waiting for data"));
        }
    }while(NULL == data.get()); // repeat till we get a known message

    return data;
}

Config::Config(const SimpleConfig &simpleCfg) :
    serialDevice_(simpleCfg.serialDevice_),
    baudRate_(simpleCfg.baudRate_),
    enableImu_(true),
    imuType_(simpleCfg.imuType_),
    enableInsPva_(true),
    enableGpsPos_(true),
    enableGpsVel_(true),
    enableRawImu_(true),
    ignoreUnknownMessages_(false),
    dtInsPva_(0.02),
    dtGpsPos_(0.2),
    dtGpsVel_(0.2),
    fixInvalidRateSettings_(false),
    imuToGpsOffset_(simpleCfg.imuToGpsOffset_),
    imuToGpsOffsetUncertainty_(0,0.0),
    enableInsOffset_(false),
    enableInsPhaseUpdate_(false),
    enableCDGPS_(false),
    enableSBAS_(false),
    enableRTK_(false),
    enableUseOfOmniStarCarrier_(false),
    enableSetImuOrientation_(false),
    enableVehicleBodyRotation_(false) {
}
Config::Config(const GpsOnlyConfig &gpsOnlyCfg) :
    serialDevice_(gpsOnlyCfg.serialDevice_),
    baudRate_(gpsOnlyCfg.baudRate_),
    enableImu_(false),
    enableInsPva_(false),
    enableGpsPos_(true),
    enableGpsVel_(true),
    enableRawImu_(false),
    ignoreUnknownMessages_(false),
    dtGpsPos_(0.05),
    dtGpsVel_(0.05),
    fixInvalidRateSettings_(false),
    enableCDGPS_(false),
    enableSBAS_(false),
    enableRTK_(false),
    enableUseOfOmniStarCarrier_(false) {
}
Config::Config() :
    serialDevice_(""),
    baudRate_(0),
    enableImu_(false),
    imuType_(""),
    enableInsPva_(false),
    enableGpsPos_(false),
    enableGpsVel_(false),
    enableRawImu_(false),
    ignoreUnknownMessages_(false),
    dtInsPva_(1.0),
    dtGpsPos_(1.0),
    dtGpsVel_(1.0),
    fixInvalidRateSettings_(false),
    imuToGpsOffset_(0,0.0),
    imuToGpsOffsetUncertainty_(0,0.0),
    enableInsOffset_(false),
    insOffset_(0,0.0),
    enableInsPhaseUpdate_(false),
    enableCDGPS_(false),
    enableSBAS_(false),
    enableRTK_(false),
    enableUseOfOmniStarCarrier_(false),
    enableSetImuOrientation_(false),
    setImuOrientation_(0),
    enableVehicleBodyRotation_(false),
    vehicleBodyRotation_(0,0.0),
    vehicleBodyRotationUncertainty_(0,0.0) {
}

bool
Config::isValid() {
    bool valid = true;
    //check serial setup
    if( 0 == serialDevice_.compare("")
        || (9600 != baudRate_
                && 19200 != baudRate_
                && 38400 != baudRate_
                && 57600 != baudRate_
                && 115200 != baudRate_
                && 230400 != baudRate_)){
        std::cerr << "serial settings invalid\n";
        valid = false;
    }

    //imu gear
    if(enableImu_
            && ( 0 == imuType_.compare("")
                || 3 != imuToGpsOffset_.size()
                || ( 3 != imuToGpsOffsetUncertainty_.size()
                    && 0  != imuToGpsOffsetUncertainty_.size()))){
        std::cerr << "imuType/imuToGpsOffset invalid\n";
        valid = false;
    }
    if(enableImu_ && enableSetImuOrientation_
            && ( 0 > setImuOrientation_
                || 6 < setImuOrientation_)){
        std::cerr << "setImuOrientation invalid\n";
        valid = false;
    }
    if(enableImu_ && enableVehicleBodyRotation_
            && ( 3 != vehicleBodyRotation_.size()
                || ( 3 != vehicleBodyRotationUncertainty_.size()
                    && 0 != vehicleBodyRotationUncertainty_.size()))){
        std::cerr << "vehicleBodyRotation invalid\n";
        valid = false;
    }

    //ins gear
    if(enableImu_ && enableInsOffset_
            && 3 != insOffset_.size()){
        std::cerr << "insOffset invalid\n";
        valid = false;
    }

    //data
    if(false == (enableInsPva_ || enableGpsPos_ || enableGpsVel_ || enableRawImu_)){
        std::cerr << "data settings invalid, you need to enable at least one message\n";
        valid = false;
    }
    if(enableRawImu_ && 0.02 > dtInsPva_){
        if(fixInvalidRateSettings_){
            std::cerr << "data rate for InsPva too high; must be 50Hz or less if RawImu is enabled. FIXED!\n";
            dtInsPva_ = 0.02;
        }else{
            std::cerr << "data rate for InsPva too high; must be 50Hz or less if RawImu is enabled\n";
            valid = false;
        }
    }
    if( (enableRawImu_ || enableInsPva_) && 0.2 > dtGpsPos_ ){
        if(fixInvalidRateSettings_){
            std::cerr << "data rate for BestGpsPos too high; must be 5Hz or less if RawImu or InsPva is enabled. FIXED!\n";
            dtGpsPos_ = 0.2;
        }else{
            std::cerr << "data rate for BestGpsPos too high; must be 5Hz or less if RawImu or InsPva is enabled\n";
            valid = false;
        }
    }
    if( (enableRawImu_ || enableInsPva_) && 0.2 > dtGpsVel_ ){
        if(fixInvalidRateSettings_){
            std::cerr << "data rate for BestGpsVel too high; must be 5Hz or less if RawImu or InsPva is enabled. FIXED!\n";
            dtGpsVel_ = 0.2;
        }else{
            std::cerr << "data rate for BestGpsVel too high; must be 5Hz or less if RawImu or InsPva is enabled\n";
            valid = false;
        }
    }

    //datarate
    double dataRate = 0.0;
    if(enableRawImu_){
        dataRate += 1.0/0.01 * 8 * sizeof (gnua::RawImuLogSB); // TODO: there is one IMU running at 200Hz, no idea how to check for it
    }
    if(enableInsPva_){
        dataRate += 1.0/dtInsPva_ * 8 * sizeof (gnua::InsPvaLogSB);
    }
    if(enableGpsPos_){
        dataRate += 1.0/dtGpsPos_ * 8 * sizeof (gnua::BestGpsPosLogB);
    }
    if(enableGpsVel_){
        dataRate += 1.0/dtGpsVel_ * 8 * sizeof (gnua::BestGpsVelLogB);
    }
    if(dataRate > (double)baudRate_){
        std::cerr << "The combined data-rate of the messages/message rates you configured excceeds the configured baud rate " << dataRate << " / " << baudRate_ << "\n";
        valid = false;
    }else if(dataRate > 0.95 * baudRate_){
        std::cerr << "The combined data-rate of the messages/message rates you configured is more than 95\% of the configured baud rate! " << dataRate << " / " << baudRate_ << "\n";
        std::cerr << "Consider using a higher baud rate (maximum 230400) or less messages or lower message-rates.";
    }

    return valid;
}

std::string
Config::toString() const{
    std::stringstream ss;
    ss << "serialDevice_: " << serialDevice_ << " ";
    ss << "baudRate_: " << baudRate_ << " ";
    ss << "enableImu_: " << enableImu_ << " ";
    ss << "imuType_: " << imuType_ << " ";
    ss << "enableInsPva_: " << enableInsPva_ << " ";
    ss << "enableGpsPos_: " << enableGpsPos_ << " ";
    ss << "enableGpsVel_: " << enableGpsVel_ << " ";
    ss << "enableRawImu_: " << enableRawImu_ << " ";
    ss << "ignoreUnknownMessages_: " << ignoreUnknownMessages_ << " ";
    ss << "dtInsPva_: " << dtInsPva_ << " ";
    ss << "dtGpsPos_: " << dtGpsPos_ << " ";
    ss << "dtGpsVel_: " << dtGpsVel_ << " ";
    ss << "fixInvalidRateSettings_: " << fixInvalidRateSettings_ << " ";
    ss << "imuToGpsOffset_: " << doubleVectorToString(imuToGpsOffset_) << " ";
    ss << "imuToGpsOffsetUncertainty_: " << doubleVectorToString(imuToGpsOffsetUncertainty_) << " ";
    ss << "enableInsOffset_: " << enableInsOffset_ << " ";
    ss << "insOffset_: " << doubleVectorToString(insOffset_) << " ";
    ss << "enableInsPhaseUpdate_: " << enableInsPhaseUpdate_ << " ";
    ss << "enableCDGPS_: " << enableCDGPS_ << " ";
    ss << "enableSBAS_: " << enableSBAS_ << " ";
    ss << "enableRTK_: " << enableRTK_ << " ";
    ss << "enableUseOfOmniStarCarrier_: " << enableUseOfOmniStarCarrier_ << " ";
    ss << "enableSetImuOrientation_: " << enableSetImuOrientation_ << " ";
    ss << "setImuOrientation_: " << setImuOrientation_ << " ";
    ss << "enableVehicleBodyRotation_: " << enableVehicleBodyRotation_ << " ";
    ss << "vehicleBodyRotation_: " << doubleVectorToString(vehicleBodyRotation_) << " ";
    ss << "vehicleBodyRotationUncertainty_: " << doubleVectorToString(vehicleBodyRotationUncertainty_);
    return ss.str();
}

bool
SimpleConfig::isValid() const {
    return 0 != serialDevice_.compare("")
        && imuType_.compare("")
        && 3 == imuToGpsOffset_.size()
        && (9600 == baudRate_
                || 19200 == baudRate_
                || 38400 == baudRate_
                || 57600 == baudRate_
                || 115200 == baudRate_
                || 230400 == baudRate_) ;
}

std::string
SimpleConfig::toString() const{
    std::stringstream ss;
    ss << "serialDevice_: " << serialDevice_ << " ";
    ss << "baudRate_: " << baudRate_ << " ";
    ss << "imuType_: " << imuType_ << " ";
    ss << "imuToGpsOffset_: " << doubleVectorToString(imuToGpsOffset_);
    return ss.str();
}

bool
GpsOnlyConfig::isValid() const {
    return 0 != serialDevice_.compare("")
        && (9600 == baudRate_
                || 19200 == baudRate_
                || 38400 == baudRate_
                || 57600 == baudRate_
                || 115200 == baudRate_
                || 230400 == baudRate_) ;
}

std::string
GpsOnlyConfig::toString() const{
    std::stringstream ss;
    ss << "serialDevice_: " << serialDevice_ << " ";
    ss << "baudRate_: " << baudRate_;
    return ss.str();
}

std::string
InsPvaData::toString() const{
    std::stringstream ss;
    ss << "InsPvaData ";
    ss << "timeStampSec " << timeStampSec << " ";
    ss << "timeStampUSec " << timeStampUSec << " ";
    ss << "gpsWeekNr " << std::fixed << std::setprecision(4) << gpsWeekNr << " ";
    ss << "secIntoWeek " << secIntoWeek << " ";
    ss << "latitude " << std::setprecision(9) << latitude << " "; // ~1/10mm resolution (equator)
    ss << "longitude " << longitude << " ";
    ss << "height " << std::setprecision(4) << height << " ";
    ss << "northVelocity " << northVelocity << " ";
    ss << "eastVelocity " << eastVelocity << " ";
    ss << "upVelocity " << upVelocity << " ";
    ss << "roll " << roll << " ";
    ss << "pitch " << pitch << " ";
    ss << "azimuth " << azimuth << " ";
    ss << statusToString(statusMessageType, statusMessage);
    return ss.str();
};

std::string
BestGpsPosData::toString() const{
    std::stringstream ss;
    ss << "BestGpsPosData ";
    ss << "timeStampSec " << timeStampSec << " ";
    ss << "timeStampUSec " << timeStampUSec << " ";
    ss << "gpsWeekNr " << gpsWeekNr << " ";
    ss << "msIntoWeek " << msIntoWeek << " ";
    ss << "solutionStatus " << solutionStatus << " ";
    ss << "positionType " << positionType << " ";
    ss << "latitude " << std::fixed << std::setprecision(9) << latitude << " "; // ~1/10mm resolution (equator)
    ss << "longitude " << longitude << " ";
    ss << "heightAMSL " << std::setprecision(4) << heightAMSL << " ";
    ss << "undulation " << undulation << " ";
    ss << "datumId " << datumId << " ";
    ss << "sigmaLatitude " << sigmaLatitude << " ";
    ss << "sigmaLongitude " << sigmaLongitude << " ";
    ss << "sigmaHeight " << sigmaHeight << " ";
    ss << "baseStationId ["
        << baseStationId[0]
        << baseStationId[1]
        << baseStationId[2]
        << baseStationId[3]
        << "] ";
    ss << "diffAge " << diffAge << " ";
    ss << "solutionAge " << solutionAge << " ";
    ss << "numObservations " << numObservations << " ";
    ss << "numL1Ranges " << numL1Ranges << " ";
    ss << "numL1RangesRTK " << numL1RangesRTK << " ";
    ss << "numL2RangesRTK " << numL2RangesRTK << " ";
    ss << statusToString(statusMessageType, statusMessage);
    return ss.str();
};

std::string
BestGpsVelData::toString() const{
    std::stringstream ss;
    ss << "BestGpsVelData ";
    ss << "timeStampSec " << timeStampSec << " ";
    ss << "timeStampUSec " << timeStampUSec << " ";
    ss << "gpsWeekNr " << gpsWeekNr << " ";
    ss << "msIntoWeek " << msIntoWeek << " ";
    ss << "solutionStatus " << solutionStatus << " ";
    ss << "velocityType " << velocityType << " ";
    ss << "latency " << std::fixed << std::setprecision(4) << latency << " ";
    ss << "diffAge " << diffAge << " ";
    ss << "horizontalSpeed " << horizontalSpeed << " ";
    ss << "trackOverGround " << trackOverGround << " ";
    ss << "verticalSpeed " << verticalSpeed << " ";
    ss << statusToString(statusMessageType, statusMessage);
    return ss.str();
};

std::string
RawImuData::toString() const{
    std::stringstream ss;
    ss << "RawImuData ";
    ss << "timeStampSec " << timeStampSec << " ";
    ss << "timeStampUSec " << timeStampUSec << " ";
    ss << "gpsWeekNr " << std::fixed << gpsWeekNr << " ";
    ss << "secIntoWeek " << std::setprecision(4) << secIntoWeek << " ";
    ss << "zDeltaV "  << std::setprecision(10)<< zDeltaV << " "; // ~1/10 LSB for best IMU
    ss << "yDeltaV " << yDeltaV << " ";
    ss << "xDeltaV " << xDeltaV << " ";
    ss << "zDeltaAng " <<  std::setprecision(11) << zDeltaAng << " ";  // ~1/10 LSB for best IMU
    ss << "yDeltaAng " << yDeltaAng << " ";
    ss << "xDeltaAng " << xDeltaAng << " ";
    ss << statusToString(statusMessageType, statusMessage);
    return ss.str();
};

std::string 
toString( StatusMessageType type ){
    switch( type )
    {
        case NoMsg       : return "None";
        case Initialising: return "Initialising";
        case Ok          : return "Ok";
        case Warning     : return "Warning";
        case Fault       : return "Fault";
        default: return "Unknown";
    }
}

std::string 
toString( GpsSolutionStatusType type ){
    switch( type ){
        case SolComputed                    : return "SolComputed";
        case InsufficientObs                : return "InsufficientObs";
        case NoConvergence                  : return "NoConvergence";
        case Singularity                    : return "Singularity";
        case CovTrace                       : return "CovTrace";
        case TestDist                       : return "TestDist";
        case ColdStart                      : return "ColdStart";
        case VHLimit                        : return "VHLimit";
        case Variance                       : return "Variance";
        case Residuals                      : return "Residuals";
        case DeltaPos                       : return "DeltaPos";
        case NegativeVar                    : return "NegativeVar";
        case ReservedGpsSolutionStatusType12: return "Reserved";
        case IntegrityWarning               : return "IntegrityWarning";
        case InsInactive                    : return "InsInactive";
        case InsAligning                    : return "InsAligning";
        case InsBad                         : return "InsBad";
        case ImuUnplugged                   : return "ImuUnplugged";
        case Pending                        : return "Pending";
        case InvalidFix                     : return "InvalidFix";
        default: return "Unknown";
    }
}

std::string 
toString( GpsPosVelType type ){
    switch( type ){
        case None                   : return "None";
        case FixedPos               : return "FixedPos";
        case FixedHeight            : return "FixedHeight";
        case ReservedGpsPosVelType3 : return "Reserved";
        case FloatConv              : return "FloatConv";
        case WideLane               : return "WideLane";
        case NarrowLane             : return "NarrowLane";
        case ReservedGpsPosVelType7 : return "Reserved";
        case DopplerVelocity        : return "DopplerVelocity";
        case ReservedGpsPosVelType9 : return "Reserved";
        case ReservedGpsPosVelType10: return "Reserved";
        case ReservedGpsPosVelType11: return "Reserved";
        case ReservedGpsPosVelType12: return "Reserved";
        case ReservedGpsPosVelType13: return "Reserved";
        case ReservedGpsPosVelType14: return "Reserved";
        case ReservedGpsPosVelType15: return "Reserved";
        case Single                 : return "Single";
        case PsrDiff                : return "PsrDiff";
        case Waas                   : return "Waas";
        case Propagated             : return "Propagated";
        case Omnistar               : return "Omnistar";
        case ReservedGpsPosVelType21: return "Reserved";
        case ReservedGpsPosVelType22: return "Reserved";
        case ReservedGpsPosVelType23: return "Reserved";
        case ReservedGpsPosVelType24: return "Reserved";
        case ReservedGpsPosVelType25: return "Reserved";
        case ReservedGpsPosVelType26: return "Reserved";
        case ReservedGpsPosVelType27: return "Reserved";
        case ReservedGpsPosVelType28: return "Reserved";
        case ReservedGpsPosVelType29: return "Reserved";
        case ReservedGpsPosVelType30: return "Reserved";
        case ReservedGpsPosVelType31: return "Reserved";
        case L1Float                : return "L1Float";
        case IonoFreeFloat          : return "IonoFreeFloat";
        case NarrowFloat            : return "NarrowFloat";
        case L1Int                  : return "L1Int";
        case WideInt                : return "WideInt";
        case NarrowInt              : return "NarrowInt";
        case RtkDirectIns           : return "RtkDirectIns";
        case Ins                    : return "Ins";
        case InsPsrSp               : return "InsPsrSp";
        case InsPsrDiff             : return "InsPsrDiff";
        case InsRtkFloat            : return "InsRtkFloat";
        case InsRtkFixed            : return "InsRtkFixed";
        case OmniStarHp             : return "OmniStarHp";
        case OmniStarXp             : return "OmniStarXp";
        case CdGps                  : return "CdGps";
        default: return "Unknown";
    }
}

} //namespace

namespace{
    void
    checkParserAssumptions(){
        // we have a fairly dodgy parser, which makes unsound assumptions about the binary layout of things in memory.
        // This gear makes sure that we fail at runtime instead of delivering garbage.
        assert((8 == CHAR_BIT) && "Unsupported size of byte: our parser won't work on this platform");
        assert((8 == sizeof(double)) && "Unsupported size of double: our parser won't work on this platform");
        assert((4 == sizeof(float)) && "Unsupported size of float: our parser won't work on this platform");

        union EndianCheck{
            uint32_t full;
            uint8_t bytes[4];
        };
        EndianCheck endianCheck;
        endianCheck.bytes[0] = 0x21;
        endianCheck.bytes[1] = 0x43;
        endianCheck.bytes[2] = 0x65;
        endianCheck.bytes[3] = 0x87;
        assert(0x87654321 == endianCheck.full && "Unsupported endianness: our parser won't work on this platform");
        assert(rawMsgSize == sizeof(union NovatelMessage)
                && "Overflow: Someone included a log definition in [NovatelMessage] which is bigger than the buffer for it"
                && "Depending on the pickiness of your hardware this might actually work, but I'll shut down on general principle");
        return;
    }

    uint16_t
    readNovatelMessage(union NovatelMessage &msg, struct timeval &timeStamp, gbxserialacfr::Serial *serial) {
        uint16_t id = gnua::InvalidLogType;
        msg.header.sb1 = 0;

        // skip everything until we get the first sync byte
        do{
            if( 1 != serial->readFull( &msg.header.sb1, 1 )){
                throw ( gua::Exception(ERROR_INFO, "Timed out while trying to read sync byte 1") );
            }
        }while( msg.header.sb1 != 0xaa );

        // get timestamp after the first byte for accuracy
        gettimeofday(&timeStamp, NULL);

        // read the second and third sync byte
        if( 1 != serial->readFull( &msg.header.sb2, 1) ){
            throw ( gua::Exception(ERROR_INFO, "Timed out while trying to read sync byte 2" ) );
        }
        if (msg.header.sb2 != 0x44 ) {
            std::stringstream ss;
            ss << "Expected sync byte 2 (0x44) got: " << hex << msg.header.sb2;
            throw ( gua::Exception(ERROR_INFO, ss.str()) );
        }
        if( 1 != serial->readFull( &msg.header.sb3, 1 )){
            throw ( gua::Exception(ERROR_INFO, "Timed out while trying to read sync byte 3") );
        }

        // figure out what binary format we have, and read the full packet
        switch( msg.header.sb3 ) {
            case 0x12: //long packet
                if( // how long is the header ?
                    1 != serial->readFull( &msg.header.headerLength, 1 )
                    // read all of the header
                    || msg.header.headerLength-4 != serial->readFull( &msg.header.msgId, msg.header.headerLength-4 )
                    // read the  message data plus 4 bytes for the crc
                    || msg.header.msgLength+4 != serial->readFull( &msg.data, msg.header.msgLength+4 )
                  ){
                    throw ( gua::Exception(ERROR_INFO, "Timed out while trying to read long packet") );
                }

                id = msg.header.msgId;

                if(0 != gnua::crc( msg.rawMessage, msg.header.msgLength+msg.header.headerLength+4 )){
                    throw ( gua::Exception(ERROR_INFO, "CRC Error" ) );
                }
                break;

            case 0x13: //short packet
                if( // read rest of the header 12 bytes - 3 bytes already read, then the actual data plus 4 bytes for the CRC
                    9 != serial->readFull( &msg.shortHeader.msgLength, 9 )
                    || msg.shortHeader.msgLength+4 != serial->readFull( &msg.shortData, msg.shortHeader.msgLength+4 )
                  ){
                    throw ( gua::Exception(ERROR_INFO, "Timed out while trying to read short packet") );
                }

                id = msg.shortHeader.msgId;

                if( 0 != gnua::crc( msg.rawMessage,msg.shortHeader.msgLength + 16 )){
                    throw ( gua::Exception(ERROR_INFO, "CRC Error" ) );
                }
                break;

            default: //bollocks
                throw ( gua::Exception(ERROR_INFO, "Unknown binary packet format" ) );
        }
        return  id;
    }


    std::auto_ptr<gna::GenericData>
    createExternalMsg(gnua::InsPvaLogSB &insPva, struct timeval &timeStamp){
        gna::InsPvaData *data = new gna::InsPvaData;
        std::auto_ptr<gna::GenericData> genericData( data );

        //data
        data->gpsWeekNr = insPva.data.gpsWeekNr;
        data->secIntoWeek = insPva.data.secIntoWeek;
        data->latitude = insPva.data.latitude;
        data->longitude = insPva.data.longitude;
        data->height = insPva.data.height;
        data->northVelocity = insPva.data.northVelocity;
        data->eastVelocity = insPva.data.eastVelocity;
        data->upVelocity = insPva.data.upVelocity;
        data->roll = insPva.data.roll;
        data->pitch = insPva.data.pitch;
        data->azimuth = insPva.data.azimuth;

        //timestamp
        data->timeStampSec = timeStamp.tv_sec;
        data->timeStampUSec = timeStamp.tv_usec;

        //status
        switch( insPva.data.insStatus ) {
            case 0:
                data->statusMessage = "Ins is inactive";
                data->statusMessageType = gna::Initialising; // TODO: how can we distinguish between the startup-behavior and a genuine fault during operation?? Timer from when we first established a connection?
                break;
            case 1:
                data->statusMessage = "Ins is aligning";
                data->statusMessageType = gna::Initialising;
                break;
            case 2:
                data->statusMessage = "Ins solution is bad";
                data->statusMessageType = gna::Warning;
                break;
            case 3:
                data->statusMessage = "Ins solution is good";
                data->statusMessageType = gna::Ok;
                break;
            case 4://fallthrough
            case 5:
                {
                    stringstream ss;
                    ss << "Reserved value?? Check NovatelSpan manual for \"" << insPva.data.insStatus << "\" as INS status";
                    data->statusMessage = ss.str();
                    data->statusMessageType = gna::Warning;
                }
                break;
            case 6:
                data->statusMessage = "Bad Ins Gps agreement";
                data->statusMessageType = gna::Warning;
                break;
            case 7:
                data->statusMessage = "Ins alignment is complete but vehicle must perform maneuvers so that the attitude can converge";
                data->statusMessageType = gna::Ok; // TODO: this is not quite Ok, check if it is usable (no sudden jumps, etc.)
                break;
            default:
                {
                    stringstream ss;
                    ss <<  "Unknown Ins Status. Check NovatelSpan manual for \"" << insPva.data.insStatus << "\" as INS status";
                    data->statusMessage = ss.str();
                    data->statusMessageType = gna::Warning;
                }
                break;
        }

        return genericData;
    }

    std::auto_ptr<gna::GenericData>
    createExternalMsg(gnua::BestGpsPosLogB &bestGpsPos, struct timeval &timeStamp){
        gna::BestGpsPosData *data = new gna::BestGpsPosData;
        std::auto_ptr<gna::GenericData> genericData( data );

        //data
        data->gpsWeekNr = bestGpsPos.header.gpsWeekNr;
        data->msIntoWeek = bestGpsPos.header.msIntoWeek;
        data->solutionStatus = externalGpsSolutionStatus( bestGpsPos.data.solutionStatus );
        data->positionType = externalGpsPosVelType( bestGpsPos.data.positionType );
        data->latitude = bestGpsPos.data.latitude;
        data->longitude = bestGpsPos.data.longitude;
        data->heightAMSL = bestGpsPos.data.heightAMSL;
        data->undulation = bestGpsPos.data.undulation;
        data->datumId = bestGpsPos.data.datumId;
        data->sigmaLatitude = bestGpsPos.data.sigmaLatitude;
        data->sigmaLongitude = bestGpsPos.data.sigmaLongitude;
        data->sigmaHeight = bestGpsPos.data.sigmaHeight;
        data->baseStationId[0] = bestGpsPos.data.baseStationId[0];
        data->baseStationId[1] = bestGpsPos.data.baseStationId[1];
        data->baseStationId[2] = bestGpsPos.data.baseStationId[2];
        data->baseStationId[3] = bestGpsPos.data.baseStationId[3];
        data->diffAge = bestGpsPos.data.diffAge;
        data->solutionAge = bestGpsPos.data.solutionAge;
        data->numObservations = bestGpsPos.data.numObservations;
        data->numL1Ranges = bestGpsPos.data.numL1Ranges;
        data->numL1RangesRTK = bestGpsPos.data.numL1RangesRTK;
        data->numL2RangesRTK = bestGpsPos.data.numL2RangesRTK;

        //time
        data->timeStampSec = timeStamp.tv_sec;
        data->timeStampUSec = timeStamp.tv_usec;

        //status
        static bool lastStatusWasGood = false;
        if(true == gnua::receiverStatusIsGood(bestGpsPos.header.receiverStatus)){
            if (true == lastStatusWasGood){
                // still all good, no need to be chatty
                data->statusMessageType = gna::NoMsg;
                data->statusMessage = "";
                lastStatusWasGood = true;
            }else{
                // we are good now, report it
                data->statusMessageType = gna::Ok;
                data->statusMessage = "all is good";
                lastStatusWasGood = true;
            }
        }else if(true == gnua::receiverStatusIsWarning(bestGpsPos.header.receiverStatus)){
            data->statusMessageType = gna::Warning;
            data->statusMessage = gnua::receiverStatusToString(bestGpsPos.header.receiverStatus);
            lastStatusWasGood = false;
        }else if(true == gnua::receiverStatusIsError(bestGpsPos.header.receiverStatus)){
            data->statusMessageType = gna::Fault;
            data->statusMessage = gnua::receiverStatusToString(bestGpsPos.header.receiverStatus);
            lastStatusWasGood = false;
        }else if(true == gnua::receiverStatusIsFatal(bestGpsPos.header.receiverStatus)){
            //ouch, need to bug out
            std::stringstream ss;
            ss << "Receiver reports hardware-error! This is not recoverable!\n"
                << "You can get additional information by sending \"log rxstatusa once\" to the receiver (minicom/other-terminal-program)!\n"
                << "GpsStatus:\n";
            ss << gnua::receiverStatusToString(bestGpsPos.header.receiverStatus);
            lastStatusWasGood = false;
            throw ( gua::HardwareException(ERROR_INFO, ss.str()) );
        }else if(true == gnua::receiverStatusIsReservedValue(bestGpsPos.header.receiverStatus)){
            //whoops
            data->statusMessageType = gna::Warning;
            std::stringstream ss;
            ss << "Got GPS status which used to be a reserved value: 0x" << hex << bestGpsPos.header.receiverStatus
                << " Please check your manual what this means, and tell us (gearbox-devel@lists.sourceforge.net) about it.\n"
                << "Thanks.\n";
            data->statusMessage = ss.str();
            lastStatusWasGood = false;
        }else{
            // Can't happen
            data->statusMessageType = gna::Warning;
            std::stringstream ss;
            ss << "Failed to decode GPS status: 0x" << hex << bestGpsPos.header.receiverStatus
                << " Please send a bug-report to gearbox-devel@lists.sourceforge.net.\n"
                << "Include this message and details about your hardware/software configuration.\n"
                << "Thanks.\n";
            data->statusMessage = ss.str();
            lastStatusWasGood = false;
        }
        return genericData;

    }

    std::auto_ptr<gna::GenericData>
    createExternalMsg(gnua::BestGpsVelLogB &bestGpsVel, struct timeval &timeStamp){
        gna::BestGpsVelData *data = new gna::BestGpsVelData;
        std::auto_ptr<gna::GenericData> genericData( data );

        //data
        data->gpsWeekNr = bestGpsVel.header.gpsWeekNr;
        data->msIntoWeek = bestGpsVel.header.msIntoWeek;
        data->solutionStatus = externalGpsSolutionStatus( bestGpsVel.data.solutionStatus );
        data->velocityType = externalGpsPosVelType( bestGpsVel.data.velocityType );
        data->latency = bestGpsVel.data.latency;
        data->diffAge = bestGpsVel.data.diffAge;
        data->horizontalSpeed = bestGpsVel.data.horizontalSpeed;
        data->trackOverGround = bestGpsVel.data.trackOverGround;
        data->verticalSpeed = bestGpsVel.data.verticalSpeed;

        //time
        data->timeStampSec = timeStamp.tv_sec;
        data->timeStampUSec = timeStamp.tv_usec;

        //status
        static bool lastStatusWasGood = false;
        if(true == gnua::receiverStatusIsGood(bestGpsVel.header.receiverStatus)){
            if (true == lastStatusWasGood){
                // still all good, no need to be chatty
                data->statusMessageType = gna::NoMsg;
                data->statusMessage = "";
                lastStatusWasGood = true;
            }else{
                // we are good now, report it
                data->statusMessageType = gna::Ok;
                data->statusMessage = "all is good";
                lastStatusWasGood = true;
            }
        }else if(true == gnua::receiverStatusIsWarning(bestGpsVel.header.receiverStatus)){
            data->statusMessageType = gna::Warning;
            data->statusMessage = gnua::receiverStatusToString(bestGpsVel.header.receiverStatus);
            lastStatusWasGood = false;
        }else if(true == gnua::receiverStatusIsError(bestGpsVel.header.receiverStatus)){
            data->statusMessageType = gna::Fault;
            data->statusMessage = gnua::receiverStatusToString(bestGpsVel.header.receiverStatus);
            lastStatusWasGood = false;
        }else if(true == gnua::receiverStatusIsFatal(bestGpsVel.header.receiverStatus)){
            //ouch, need to bug out
            std::stringstream ss;
            ss << "Receiver reports hardware-error! This is not recoverable!\n"
                << "You can get additional information by sending \"log rxstatusa once\" to the receiver (minicom/other-terminal-program)!\n"
                << "GpsStatus:\n";
            ss << gnua::receiverStatusToString(bestGpsVel.header.receiverStatus);
            lastStatusWasGood = false;
            throw ( gua::HardwareException(ERROR_INFO, ss.str()) );
        }else if(true == gnua::receiverStatusIsReservedValue(bestGpsVel.header.receiverStatus)){
            //whoops
            data->statusMessageType = gna::Warning;
            std::stringstream ss;
            ss << "Got GPS status which used to be a reserved value: 0x" << hex << bestGpsVel.header.receiverStatus
                << " Please check your manual what this means, and tell us (gearbox-devel@lists.sourceforge.net) about it.\n"
                << "Thanks.\n";
            data->statusMessage = ss.str();
            lastStatusWasGood = false;
        }else{
            // Can't happen
            data->statusMessageType = gna::Warning;
            std::stringstream ss;
            ss << "Failed to decode GPS status: 0x" << hex << bestGpsVel.header.receiverStatus
                << " Please send a bug-report to gearbox-devel@lists.sourceforge.net.\n"
                << "Include this message and details about your hardware/software configuration.\n"
                << "Thanks.\n";
            data->statusMessage = ss.str();
            lastStatusWasGood = false;
        }




        return genericData;
    }

    std::auto_ptr<gna::GenericData>
    createExternalMsg(gnua::RawImuLogSB &rawImu, struct timeval &timeStamp, gnua::ImuDecoder *imuDecoder){
        gna::RawImuData *data = new gna::RawImuData;
        std::auto_ptr<gna::GenericData> genericData( data );

        //data
        data->gpsWeekNr = rawImu.data.gpsWeekNr;
        data->secIntoWeek = rawImu.data.secIntoWeek;
        //accels
        data->zDeltaV = imuDecoder->accelCnt2MperSec(rawImu.data.zAccelCnt);
        data->yDeltaV = -1.0*imuDecoder->accelCnt2MperSec(rawImu.data.yNegativAccelCnt);
        data->xDeltaV = imuDecoder->accelCnt2MperSec(rawImu.data.xAccelCnt);
        //gyros
        data->zDeltaAng = imuDecoder->gyroCnt2Rad(rawImu.data.zGyroCnt);
        data->yDeltaAng = -1.0*imuDecoder->gyroCnt2Rad(rawImu.data.yNegativGyroCnt);
        data->xDeltaAng = imuDecoder->gyroCnt2Rad(rawImu.data.xGyroCnt);

        //time
        data->timeStampSec = timeStamp.tv_sec;
        data->timeStampUSec = timeStamp.tv_usec;

        //status
        static bool lastStatusWasGood = false;
        if(true == imuDecoder->statusIsGood(rawImu.data.imuStatus)){
            if (true == lastStatusWasGood){
                // still all good, no need to be chatty
                data->statusMessageType = gna::NoMsg;
                data->statusMessage = "";
                lastStatusWasGood = true;
            }else{
                // we are good now, report it
                data->statusMessageType = gna::Ok;
                data->statusMessage = "all is good";
                lastStatusWasGood = true;
            }
        }else{
            //whoops
            data->statusMessageType = gna::Fault; // warning?
            data->statusMessage = imuDecoder->statusToString(rawImu.data.imuStatus);
            lastStatusWasGood = false;
        }
        return genericData;
    }

    enum gna::GpsSolutionStatusType externalGpsSolutionStatus(uint32_t novatelGpsSolutionStatus){
        if( novatelGpsSolutionStatus>=0 && novatelGpsSolutionStatus<=19  )
           return static_cast<gna::GpsSolutionStatusType>(novatelGpsSolutionStatus);
        else
           return gna::UnknownGpsSolutionStatusType;
    }

    enum gna::GpsPosVelType externalGpsPosVelType(uint32_t novatelGpsPosVelType){
        // There are unused values between (and including) 35 to 47 and 57 to 63
        if( (novatelGpsPosVelType>=0  && novatelGpsPosVelType<=34 ) ||
            (novatelGpsPosVelType>=48 && novatelGpsPosVelType<=56 ) ||
            (novatelGpsPosVelType>=64 && novatelGpsPosVelType<=66 ) )
           return static_cast<gna::GpsPosVelType>(novatelGpsPosVelType);
        else
           return gna::UnknownGpsPosVelType;
    }

    std::string doubleVectorToString(const vector<double > &vec, const std::string seperator){
        std::stringstream ss;
        int max = vec.size();
        ss << "[";
        for (int i=0; i<max; i++){
            ss << vec[i] << seperator;
        }
        ss << "]";
        return ss.str();
    }

    std::string statusToString(gna::StatusMessageType statusMessageType, std::string statusMessage){
        std::stringstream ss;
        switch(statusMessageType){
            case gna::NoMsg:
                   ss << "NoMsg" << " ";
                   break;
            case  gna::Initialising:
                   ss << "Initialising" << " ";
                   break;
            case  gna::Ok:
                   ss << "Ok" << " ";
                   break;
            case  gna::Warning:
                   ss << "Warning" << " ";
                   break;
            case gna::Fault:
                   ss << "Fault" << " ";
                   break;
            default:
                   ss << "UnknownStatus" << " ";
        }
        ss << "[" << statusMessage << "]";
        return ss.str();
    }
}//namespace
