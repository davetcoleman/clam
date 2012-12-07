/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <cstring>

#include <gbxutilacfr/gbxutilacfr.h>
#include <gbxsickacfr/gbxiceutilacfr/gbxiceutilacfr.h>
#include "driver.h"

using namespace std;

namespace gbxsickacfr {

namespace {

    class NoRxMsgException : public std::exception
    {
        std::string  message_;
    public:
        NoRxMsgException(const char *message)
            : message_(message) {}
        NoRxMsgException(const std::string &message)
            : message_(message) {}
        ~NoRxMsgException()throw(){}
        virtual const char* what() const throw() { return message_.c_str(); }
    };

    class NackReceivedException : public std::exception
    {
        std::string  message_;
    public:
        NackReceivedException(const char *message)
            : message_(message) {}
        NackReceivedException(const std::string &message)
            : message_(message) {}
        ~NackReceivedException()throw(){}
        virtual const char* what() const throw() { return message_.c_str(); }
    };

    class RxMsgIsErrorException : public std::exception
    {
        std::string  message_;
    public:
        RxMsgIsErrorException(const char *message)
            : message_(message) {}
        RxMsgIsErrorException(const std::string &message)
            : message_(message) {}
        ~RxMsgIsErrorException()throw(){}
        virtual const char* what() const throw() { return message_.c_str(); }
    };
}
////////////////////////

Config::Config() :   
    baudRate(38400),
    minRange(0.0),
    maxRange(0.0),
    fieldOfView(0.0),
    startAngle(0.0),
    numberOfSamples(0)
{
}

bool
Config::isValid() const
{
    // Don't bother verifying device, the user will find out soon enough when the Driver bitches.
    if ( !( baudRate == 9600    ||
            baudRate == 19200   ||
            baudRate == 38400   ||
            baudRate == 500000 ) )
    {
        return false;
    }
    if ( minRange < 0.0 ) return false;
    if ( maxRange <= 0.0 ) return false;
    if ( fieldOfView <= 0.0 || fieldOfView > DEG2RAD(360.0) ) return false;
    if ( startAngle <= DEG2RAD(-360.0) || startAngle > DEG2RAD(360.0) ) return false;
    if ( numberOfSamples <= 0 ) return false;

    return true;
}

std::string
Config::toString() const
{
    std::stringstream ss;
    ss << "Laser driver config: device="<<device<<", baudRate="<<baudRate<<", minr="<<minRange<<", maxr="<<maxRange<<", fov="<<RAD2DEG(fieldOfView)<<"deg, start="<<RAD2DEG(startAngle)<<"deg, num="<<numberOfSamples;
    return ss.str();
}

bool 
Config::operator==( const Config & other )
{
    return (minRange==other.minRange && maxRange==other.maxRange && fieldOfView==other.fieldOfView 
         && startAngle==other.startAngle && numberOfSamples==other.numberOfSamples);
}

bool 
Config::operator!=( const Config & other )
{
    return (minRange!=other.minRange || maxRange!=other.maxRange || fieldOfView!=other.fieldOfView 
         || startAngle!=other.startAngle || numberOfSamples!=other.numberOfSamples);
}

////////////////////////

Driver::Driver( const Config &config, gbxutilacfr::Tracer& tracer, gbxutilacfr::Status& status )
    : config_(config),
      tracer_(tracer),
      status_(status)
{
    if ( !config.isValid() )
    {
        stringstream ss;
        ss << __func__ << "(): Invalid config: " << config.toString();
        throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
    }

    stringstream ssDebug;
    ssDebug << "Connecting to laser on serial port " << config_.device;
    tracer_.debug( ssDebug.str() );

    const int MAX_TRIES=3;
    for ( int i=0; i < MAX_TRIES; i++ )
    {
        stringstream tryString;
        tryString << "Connection attempt " << i+1 << " of " << MAX_TRIES << ": ";
        try {

            serialHandler_.reset(0);
            serialHandler_.reset( new SerialHandler( config_.device, tracer, status ) );
            initLaser();
            break;
        }
        catch ( const RxMsgIsErrorException &e )
        {
            std::string errorLog = errorConditions();
            stringstream ss;
            ss << e.what() << endl << "Laser error log: " << errorLog;
            if ( i == MAX_TRIES-1 )
                throw RxMsgIsErrorException( ss.str() );
            else
                tracer_.warning( tryString.str() + ss.str() );
        }
        catch ( const std::exception &e )
        {
            stringstream ss;
            ss << "during initLaser(): " << e.what();
            tracer_.warning( tryString.str() + ss.str() );
            if ( i == MAX_TRIES-1 )
                throw;
        }
    }
}

bool
Driver::waitForRxMsgType( uChar type, TimedLmsRxMsg &rxMsg, int maxWaitMs )
{
    gbxiceutilacfr::Timer waitTimer;
    
    while ( true )
    {
        int ret = serialHandler_->getNextRxMsg( rxMsg, maxWaitMs );
        if ( ret == 0 )
        {
            // got rxMsg
            if ( rxMsg.msg->type == type )
                return true;

            // Got a rxMsg, but not of the type we were expecting...
            //
            // In general, print out a warning message.
            // However, don't print out a warning if it's an ACK/NACK, because ACK/NACK don't
            // have checksums so they're likely to be found inside a failed-checksum message when a bit
            // gets flipped during transmission.
            if ( rxMsg.msg->type != ACK &&
                 rxMsg.msg->type != NACK )
            {
                stringstream ss;
                ss << "Driver::waitForRxMsgType(): While waiting for " << cmdToString(type) << ", received unexpected rxMsg: " << toString(*(rxMsg.msg));
                tracer_.warning( ss.str() );
            }

            if ( waitTimer.elapsedMs() > maxWaitMs )
            {
                // waited too long
                return false;
            }
        }
        else if ( ret == -1 )
        {
            // timed out on buffer
            return false;
        }
        else
        {
            stringstream ss; ss << "Weird return code from getAndPopNext: " << ret;
            throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
        }
    }
}

bool
Driver::waitForAckOrNack( bool &receivedAck )
{
    const int maxWaitMs = 1000;

    gbxiceutilacfr::Timer waitTimer;

    while ( true )
    {
        TimedLmsRxMsg timedRxMsg;

        int ret = serialHandler_->getNextRxMsg( timedRxMsg, maxWaitMs );
        if ( ret == 0 )
        {
            // got timedRxMsg
            if ( timedRxMsg.msg->type == ACK || timedRxMsg.msg->type == NACK )
            {
                receivedAck = (timedRxMsg.msg->type == ACK);
                return true;
            }
            else
            {
                stringstream ss;
                ss << "Driver::waitForAckOrNack(): Received unexpected rxMsg: " << toString(*(timedRxMsg.msg));
                tracer_.warning( ss.str() );
            }

            double waitTimeSec = waitTimer.elapsedSec();
            if ( waitTimeSec > maxWaitMs/1000.0 )
            {
                return false;
            }
        }
        else if ( ret == -1 )
        {
            tracer_.debug( "Driver: waitForAckOrNack(): timed out.", 3 );
            return false;
        }
        else
        {
            stringstream ss;
            ss << "Weird return code from serialHandler_->getNextRxMsg: " << ret;
            throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
        }
    }    
}

LmsRxMsgPtr
Driver::askLaserForStatusData()
{
    constructStatusRequest( commandAndData_ );

    TimedLmsRxMsg rxMsg = sendAndExpectRxMsg( commandAndData_ );
    return rxMsg.msg;
}

LmsRxMsgPtr
Driver::askLaserForConfigData()
{
    constructConfigurationRequest( commandAndData_ );
    try {
        TimedLmsRxMsg rxMsg = sendAndExpectRxMsg( commandAndData_ );
        return rxMsg.msg;
    }
    catch ( NoRxMsgException &e )
    {
        stringstream ss;
        ss << "While trying to askLaserForConfigData(): " << e.what();
        throw NoRxMsgException( ss.str() );
    }
}

uChar
Driver::desiredMeasuredValueUnit()
{
    if ( (int)(round(config_.maxRange)) == 80 )
    {
        return MEASURED_VALUE_UNIT_CM;
    }
    else if ( (int)(round(config_.maxRange)) == 8 )
    {
        return MEASURED_VALUE_UNIT_MM;
    }
    else
    {
        stringstream ss;
        ss << "Unknown linear resolution: " << config_.maxRange;
        throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
    }
}

uint16_t
Driver::desiredAngularResolution()
{
    double angleIncrement = config_.fieldOfView / (config_.numberOfSamples-1);
    int angleIncrementInHundredthDegrees = (int)round(100.0 * angleIncrement*180.0/M_PI);

    assert( angleIncrementInHundredthDegrees == ANGULAR_RESOLUTION_1_0_DEG ||
            angleIncrementInHundredthDegrees == ANGULAR_RESOLUTION_0_5_DEG ||
            angleIncrementInHundredthDegrees == ANGULAR_RESOLUTION_0_25_DEG );
    return (uint16_t)angleIncrementInHundredthDegrees;
}

bool 
Driver::isAsDesired( const LmsConfigurationData &lmsConfig )
{
    // This thing has a default constructor which will fill out most things
    return ( lmsConfig == desiredConfiguration() );
}

LmsConfigurationData
Driver::desiredConfiguration()
{
    // Default constructor sets up reasonable defaults which we then modify
    LmsConfigurationData c;
    
    c.measuringMode = MEASURING_MODE_8m80m_REFLECTOR8LEVELS;
    c.measuredValueUnit = desiredMeasuredValueUnit();

    // AlexB: It's not entirely clear, but from reading the SICK manual
    //        it looks like perhaps setting availability to 0x01 may
    //        help with dazzle (ie sunlight interfering with the laser).
    //        I haven't had a chance to test it though, so I'm not game
    //        to set it.
    cout<<"TRACE(driver.cpp): TODO: set availability?" << endl;
    // c.availability = 0x01;

    return c;
}

std::string
Driver::errorConditions()
{
    try {
        tracer_.debug( "Driver: Checking error conditions." );
        constructRequestErrorMessage( commandAndData_ );
        const bool ignoreErrorConditions = true;
        TimedLmsRxMsg errorRxMsg = sendAndExpectRxMsg( commandAndData_, ignoreErrorConditions );
        return toString( *(errorRxMsg.msg) );
    }
    catch ( std::exception &e )
    {
        stringstream ss;
        ss << "Driver: Caught exception while getting error conditions: " << e.what();
        tracer_.debug( ss.str() );
        throw;
    }
}

void
Driver::setBaudRate( int baudRate )
{
    // Tell the laser to switch
    constructRequestBaudRate( commandAndData_, baudRate );
    sendAndExpectRxMsg( commandAndData_ );
    // And switch myself
    serialHandler_->setBaudRate( config_.baudRate );
}

int
Driver::guessLaserBaudRate()
{
    // Guess our current configuration first: maybe the laser driver was re-started.
    std::vector<int> baudRates;
    baudRates.push_back( config_.baudRate );
    if ( config_.baudRate != 9600 ) baudRates.push_back( 9600 );
    if ( config_.baudRate != 19200 ) baudRates.push_back( 19200 );
    if ( config_.baudRate != 38400 ) baudRates.push_back( 38400 );
    if ( config_.baudRate != 500000 ) baudRates.push_back( 500000 );
    
    for ( size_t baudRateI=0; baudRateI < baudRates.size(); baudRateI++ )
    {
        stringstream ss;
        ss << "Driver: Trying to connect at " << baudRates[baudRateI] << " baud.";
        tracer_.info( ss.str() );

        // Switch my local serial port
        serialHandler_->setBaudRate( baudRates[baudRateI] );
            
        try {
            stringstream ss;
            ss <<"Driver: Trying to get laser status with baudrate " << baudRates[baudRateI];
            tracer_.debug( ss.str() );
            askLaserForStatusData();
            return baudRates[baudRateI];
        }
        catch ( const NoRxMsgException &e )
        {
            stringstream ss;
            ss << "Driver::guessLaserBaudRate(): failed: " << e.what();
            tracer_.debug( ss.str() );
        }
    } // end loop over baud rates

    throw gbxutilacfr::Exception( ERROR_INFO, "Is the laser connected and powered on?  Failed to detect laser baud rate." );
}

void
Driver::initLaser()
{
    int currentBaudRate = guessLaserBaudRate();
    tracer_.debug( "Driver: Guessed the baud rate!" );

    //
    // Turn continuous mode off
    //
    tracer_.debug("Driver: Turning continuous mode off");
    constructRequestMeasuredOnRequestMode( commandAndData_ );
    sendAndExpectRxMsg( commandAndData_ );

    //
    // Set Desired BaudRate
    //
    tracer_.debug("Driver: Telling the laser to switch to the desired baud rate");
    if ( currentBaudRate != config_.baudRate )
    {
        setBaudRate( config_.baudRate );
    }

    // Gather info about the SICK
    stringstream ssInfo;
    ssInfo << "Laser info prior to initialisation:" << endl;

    //
    // Make note of error log
    //
    ssInfo << errorConditions() << endl;

    //
    // Check operating data counters
    //
    tracer_.debug("Driver: Checking operating data counters");
    constructRequestOperatingDataCounter( commandAndData_ );
    TimedLmsRxMsg counterRxMsg = sendAndExpectRxMsg( commandAndData_ );
    ssInfo << "OperatingDataCounter: " << toString(counterRxMsg.msg) << endl;

    //
    // Get status
    //
    tracer_.debug("Driver: Checking status");
    LmsRxMsgPtr statusRxMsg = askLaserForStatusData();
    LmsStatusRxMsgData *statusRxMsgData = 
        dynamic_cast<LmsStatusRxMsgData*>(statusRxMsg->data.get());
    assert( statusRxMsgData != NULL );
    ssInfo << "Status: " << statusRxMsgData->toString() << endl;

    LmsRxMsgPtr configRxMsg = askLaserForConfigData();
    LmsConfigurationData *lmsConfig = 
        dynamic_cast<LmsConfigurationData*>(configRxMsg->data.get());
    assert( lmsConfig != NULL );
    ssInfo << "Config: " << configRxMsg->data->toString() << endl;

    tracer_.info( ssInfo.str() );

    //
    // Enter installation mode
    //
    constructRequestInstallationMode( commandAndData_ );
    sendAndExpectRxMsg( commandAndData_ );

    //
    // Configure the thing if we have to
    //
    if ( !isAsDesired( *lmsConfig ) )
    {
        tracer_.info( "Driver: Have to reconfigure the laser..." );

        constructConfigurationCommand( desiredConfiguration(),
                                       commandAndData_ );
        TimedLmsRxMsg configCmdRxMsg = sendAndExpectRxMsg( commandAndData_ );
        LmsConfigurationRxMsgData *configCmdRxMsgData = 
            dynamic_cast<LmsConfigurationRxMsgData*>(configCmdRxMsg.msg->data.get());
        if ( !isAsDesired( configCmdRxMsgData->config ) )
        {
            stringstream ss;
            ss << "Error configuring SICK:  Config after configuration not what we expect.  Asked for: " << desiredConfiguration().toString() << endl << "got: " << configCmdRxMsgData->toString();
            throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
        }
    }

    //
    // Configure the angular resolution
    //
    const uint16_t desiredScanningAngle = 180;
    constructSwitchVariant( desiredScanningAngle,
                            desiredAngularResolution(),
                            commandAndData_ );
    TimedLmsRxMsg angRxMsg = sendAndExpectRxMsg( commandAndData_ );
    LmsSwitchVariantRxMsgData *angRxMsgData =
        dynamic_cast<LmsSwitchVariantRxMsgData*>(angRxMsg.msg->data.get());
    assert( angRxMsgData != NULL );
    if ( !( angRxMsgData->scanningAngle == desiredScanningAngle &&
            angRxMsgData->angularResolution == desiredAngularResolution() ) )
    {
            stringstream ss;
            ss << "Error configuring SICK variant:  Variant after configuration not what we expect: " << angRxMsgData->toString();
            throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );        
    }
    
    //
    // Start continuous mode
    //
    constructRequestContinuousMode( commandAndData_ );
    TimedLmsRxMsg contRxMsg = sendAndExpectRxMsg( commandAndData_ );

    tracer_.info( "Driver: enabled continuous mode, laser is running." );
}

TimedLmsRxMsg
Driver::sendAndExpectRxMsg( const std::vector<uChar> &commandAndData, bool ignoreErrorConditions )
{
    constructTelegram( telegramBuffer_, commandAndData );

    assert( commandAndData.size()>0 && "constructTelegram should give telegram with non-zero size." );
    const uChar command = commandAndData[0];

    stringstream ss;
    ss << "Driver: requesting send "<<cmdToString(command)<<endl<<"  telegram: " 
       << toHexString(telegramBuffer_);
    tracer_.debug( ss.str(), 3 );

    serialHandler_->send( telegramBuffer_ );

    bool isAck;
    bool receivedAckOrNack = waitForAckOrNack( isAck );
    if ( !receivedAckOrNack )
    {
        throw NoRxMsgException( "No ACK/NACK to command "+cmdToString(command) );
    }
    if ( !isAck )
    {
        throw NackReceivedException( "Received NACK for command "+cmdToString(command) );
    }

    int timeoutMs = 1000;
    if ( command == CMD_CONFIGURE_LMS )
    {
        // This takes a long time to complete
        timeoutMs = 15000;
    }

    TimedLmsRxMsg rxMsg;
    bool received = waitForRxMsgType( ack(command), rxMsg, timeoutMs );
    if ( !received )
    {
        throw NoRxMsgException( "No rxMsg to command "+cmdToString(command) );
    }

    if ( !ignoreErrorConditions && rxMsg.msg->isError() )
    {
        throw RxMsgIsErrorException( "RxMsg contains an error condition: "+toString(rxMsg.msg) );
    }
    if ( rxMsg.msg->isWarn() )
    {
        stringstream ss;
        tracer_.warning( "RxMsg contains warning: "+toString(rxMsg.msg) );
    }

    return rxMsg;
}

void 
Driver::read( Data &data )
{
    TimedLmsRxMsg rxMsg;

    // This timeout is greater than the scan inter-arrival time for all baudrates.
    const int timeoutMs = 1000;
    bool received = waitForRxMsgType( ACK_REQUEST_MEASURED_VALUES, rxMsg, timeoutMs );
    if ( !received )
    {
        throw gbxutilacfr::Exception( ERROR_INFO, "No scan received." );
    }


    LmsMeasurementData *measuredData = (LmsMeasurementData*)rxMsg.msg->data.get();

    if ( rxMsg.msg->isError() )
    {
        std::string errorLog = errorConditions();
        stringstream ss;
        ss << "Scan data indicates errors: " << toString(rxMsg.msg) << endl << "Laser error log: " << errorLog;
        throw RxMsgIsErrorException( ss.str() );        
    }
    else if ( rxMsg.msg->isWarn() )
    {
        data.haveWarnings = true;
        stringstream ss;
        ss << "Scan data indicates warnings: " << toString(rxMsg.msg);
        data.warnings = ss.str();
    }

    memcpy( &(data.ranges[0]), &(measuredData->ranges[0]), measuredData->ranges.size()*sizeof(float) );
    memcpy( &(data.intensities[0]), &(measuredData->intensities[0]), measuredData->intensities.size()*sizeof(unsigned char) );
    data.timeStampSec = rxMsg.timeStampSec;
    data.timeStampUsec = rxMsg.timeStampUsec;
}

} // namespace
