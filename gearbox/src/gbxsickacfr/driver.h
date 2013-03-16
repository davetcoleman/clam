/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_SICK_ACFR_H
#define GBX_SICK_ACFR_H

#include <gbxsickacfr/serialhandler.h>
#include <gbxutilacfr/tracer.h>
#include <gbxutilacfr/status.h>
#include <memory>

namespace gbxsickacfr {

//! Configuration structure
class Config
{   
public:
    Config();
    bool isValid() const;
    std::string toString() const;
    bool operator==( const Config & other );
    bool operator!=( const Config & other );

    //! Serial device. e.g. "/dev/ttyS0"
    std::string device;
    //! Baud rate
    int baudRate;
    //! minimum range [m]
    double minRange;
    //! maximum range [m]
    double maxRange;
    //! field of viewe [rad]
    double fieldOfView;
    //! starting angle [rad]
    double startAngle;
    //! number of samples in a scan
    int    numberOfSamples;
};

//! Data structure returned by read()
class Data
{
public:
    Data()
        : haveWarnings(false)
        {}

    float         *ranges;
    unsigned char *intensities;
    int            timeStampSec;
    int            timeStampUsec;
    bool           haveWarnings;
    //! if 'haveWarnings' is set, 'warnings' will contain diagnostic information.
    std::string    warnings;
};

//! SICK driver.
class Driver
{

public: 

    //! Constructor
    //!
    //! gbxutilacfr::Tracer and gbxutilacfr::Status allow
    //! (human-readable and machine-readable respectively) external
    //! monitorining of the driver's internal state.
    Driver( const Config        &config,
            gbxutilacfr::Tracer &tracer,
            gbxutilacfr::Status &status );

    //! Blocks till new data is available, but times out (and throws a gbxutilacfr::Exception)
    //! if it has waited an abnormally long time without receiving a scan. 
    //!
    //! The ranges and intensities in 'data' are expected to have been pre-sized correctly.
    //!
    //! Throws gbxutilacfr::Exception's on un-recoverable faults.
    //!
    void read( Data &data );

private: 

    // Waits up to maxWaitMs for a rxMsg of a particular type.
    // Returns true iff it got the rxMsg it wanted.
    bool waitForRxMsgType( uChar type, TimedLmsRxMsg &rxMsg, int maxWaitMs );
    // Returns: true if ack or nack received.
    // (and sets receivedAck: true = ACK, false=NACK)
    bool waitForAckOrNack( bool &receivedAck );

    LmsRxMsgPtr askLaserForStatusData();
    LmsRxMsgPtr askLaserForConfigData();

    LmsConfigurationData desiredConfiguration();
    bool isAsDesired( const LmsConfigurationData &lmsConfig );

    int guessLaserBaudRate();

    // Connects to the laser, sets params, and starts continuous mode.
    void initLaser();

    TimedLmsRxMsg sendAndExpectRxMsg( const std::vector<uChar> &commandAndData,
                                      bool ignoreErrorConditions=false );

    std::string errorConditions();

    uChar desiredMeasuredValueUnit();
    uint16_t desiredAngularResolution();

    void setBaudRate( int baudRate );

    Config config_;

    std::auto_ptr<SerialHandler> serialHandler_;

    std::vector<uChar> commandAndData_;
    std::vector<uChar> telegramBuffer_;

    gbxutilacfr::Tracer& tracer_;
    gbxutilacfr::Status& status_;
};

} // namespace

#endif
