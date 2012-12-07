/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_OCEANSERVER_READER_H
#define GBX_OCEANSERVER_READER_H

#include <gbxserialacfr/serial.h>
#include <gbxutilacfr/tracer.h>

#include <gbxsmartbatteryacfr/oceanserverparser.h>


namespace gbxsmartbatteryacfr
{  
    
//!
//! Class to read data from the oceanserver battery system: 
//! (1) connects to the serial port 
//! (2) reads from the serial port and parses data
//!
//! @author Tobias Kaupp
//!
class OceanServerReader
{
public:
    
    //! Connects to the serial port (e.g. /dev/ttyS0) and checks
    //! whether we are connected to an OceanServer system.
    //! Throws a HardwareReadingException if any of the above goes wrong.
    OceanServerReader( const std::string   &serialPort,
                       gbxutilacfr::Tracer &tracer );
    
    //! Reads an OceanServerSystem record
    //! May throw HardwareReadingExceptions and ParsingExceptions
    void read( OceanServerSystem &system );

    //! Resets the reader:
    //!  - tells the OceanServer system to spit out hex data
    //!  - tries to find the beginning of a new record
    void reset();
    

private:   
    
    gbxserialacfr::Serial serial_;
    gbxutilacfr::Tracer& tracer_;
    gbxsmartbatteryacfr::OceanServerParser parser_;
    
    void checkConnection();
    std::string tryToReadLineFromSerialPort();
    
    std::string beginningRecordLine_;
    bool firstTime_;
};

} // namespace

#endif
