/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_OCEANSERVER_H
#define GBX_OCEANSERVER_H

#include <memory>
#include <gbxutilacfr/tracer.h>
#include <gbxsmartbatteryacfr/oceanserverreader.h>
#include <gbxsickacfr/gbxiceutilacfr/store.h>
#include <gbxsickacfr/gbxiceutilacfr/safethread.h>

using namespace std;

namespace gbxsmartbatteryacfr {

//!
//! Thread which reads data from an OceanServer battery system.
//! Wraps up all the logic required to read from the system and
//! maintain some incremental internal data storage.
//! Also handles all ParsingExceptions.
//!
//! @author Tobias Kaupp
//!
class OceanServer : public gbxiceutilacfr::SafeThread
{    
public:
    
    //! Initialises an OceanServerReader
    //! May throw a HardwareReadingException
    OceanServer( const std::string      &port,
                 gbxutilacfr::Tracer    &tracer);

    //! Access to the current data
    //! May return an empty record ( check with isEmpty() )
    void getData( gbxsmartbatteryacfr::OceanServerSystem &data );

    //! Returns true if there is some non-empty data available
    bool haveData();

    //! The main thread function, inherited from SubsystemThread
    //! Reads data from OceanServer and incrementally updates internal storage
    //! May throw gbxutilacfr::Exception
    virtual void walk();
    
private:

    gbxiceutilacfr::Store<gbxsmartbatteryacfr::OceanServerSystem> dataStore_;
    gbxutilacfr::Tracer& tracer_;
    auto_ptr<gbxsmartbatteryacfr::OceanServerReader> reader_;
    
    int exceptionCounter_;
    std::string exceptionString_;
    
};

} //namespace

#endif



