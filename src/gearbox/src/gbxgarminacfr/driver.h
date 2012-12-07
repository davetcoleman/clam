/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Duncan Mercer, Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXGARMINACFR_DRIVER_H
#define GBXGARMINACFR_DRIVER_H

#include <gbxserialacfr/serial.h>
#include <gbxutilacfr/tracer.h>
#include <gbxutilacfr/status.h>
#include <memory>
#include <gbxgarminacfr/nmeamessages.h>

namespace gbxgarminacfr {

//! Configuration structure
class Config
{   
public:
    Config() :
        protocol("Garmin"),
        readGga(true),
        readVtg(true),
        readRme(true),
        readRmc(false),
        ignoreUnknown(false) {};

    //! Returns true if the configuration is sane. Checks include:
    bool isValid( std::string &reason ) const;

    //! Returns human-readable configuration description.
    std::string toString() const;

    //! Serial device. e.g. "/dev/ttyS0"
    std::string device;

    //! Serial Protocol: Garmin or NMEA
    std::string protocol;

    //! Read GPGGA sentence
    bool readGga;
    //! Read GPVTG sentence
    bool readVtg;
    //! Read PGRME sentence
    bool readRme; 
    //! Read GPRMC sentence
    bool readRmc;

    //! Ignore unknown messages. This driver tries to turn off all messages and then explicitely enables
    //! the ones it understands. But with some devices this does not work and many messages types are received. 
    //! When ignoreUnknown is set to TRUE the driver quietly ignores the messages it does not understand.
    bool ignoreUnknown;
};

/*! 
Garmin GPS driver.

All Garmin receivers understand the latest NMEA standard which is called: 0183 version 2.0. 

This standard dictates a transfer rate of 4800 baud.

This driver can read only the following messages (sentences):
- GPGGA: fix data 
- PGRME: (estimated error) - not sent if set to 0183 1.5 (garmin-specific)
- GPVTG: vector track and speed over ground 
- GPRMC: known as the "Recommended Minimum" sentence, is the most
         common sentence transmitted by GPS devices. This one sentence
         contains nearly everything a GPS application needs: latitude,
         longitude, speed, bearing, satellite-derived time, fix status
         and magnetic variation.

Processing of individual messages can be disabled in the Config structure.

Note that when fixType contained in the GPGGA is Invalid, all other data in all messages
except the time stamps are meaningless.

Referennces:
- http://en.wikipedia.org/wiki/NMEA
- http://www.gpsinformation.org/dale/interface.htm
*/
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
            gbxutilacfr::Status &status,
            int                  serialDebugLevel = 0 );

    ~Driver();

/*! 
Blocks till new data is available.

Throws gbxutilacfr::Exception when a problem is encountered.

@verbatim
std::auto_ptr<gbxgarminacfr::GenericData> data;

try {
    data = device->read();
}
catch ( const std::exception& e ) {
    cout <<"Test: Failed to read data: "<<e.what()<<endl;
} 
@endverbatim
*/
    std::auto_ptr<GenericData> read();

private:

    void init();
    void enableDevice();
    void disableDevice();
    
    std::auto_ptr<gbxserialacfr::Serial> serial_;

    Config config_;
    gbxutilacfr::Tracer& tracer_;
    gbxutilacfr::Status& status_;
};

} // namespace

#endif
