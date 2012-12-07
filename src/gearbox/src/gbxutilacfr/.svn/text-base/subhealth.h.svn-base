/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_SUBSYSTEM_HEALTH_H
#define GBXUTILACFR_SUBSYSTEM_HEALTH_H

#if defined (WIN32)
    #if defined (GBXUTILACFR_STATIC)
        #define GBXUTILACFR_EXPORT
    #elif defined (GBXUTILACFR_EXPORTS)
        #define GBXUTILACFR_EXPORT       __declspec (dllexport)
    #else
        #define GBXUTILACFR_EXPORT       __declspec (dllimport)
    #endif
#else
    #define GBXUTILACFR_EXPORT
#endif

#include <gbxutilacfr/status.h>

namespace gbxutilacfr {

//!
//! @brief Convenience class which allows feedback on the health of a subsystem.
//!
//! @par Overview
//!
//! Provides a convenient interface for setting health information for one subsystem.
//!
//! @sa Status, SubsystemStatus
//!
class GBXUTILACFR_EXPORT SubHealth
{

public:
    //! Sets a reference to the system Status and this subsystem's name.
    //! Adds this subsystem to the system.
    SubHealth( Status& sysStatus, const std::string& subsysName ) :
        status_(sysStatus),
        subsysName_(subsysName)
    {};

    //! Passes this information to the system Status.
    void heartbeat() { status_.heartbeat( subsysName_ ); };

    //! Passes this information to the system Status.
    void message( const std::string& message ) { status_.message( subsysName_, message ); };

    //! Passes this information to the system Status.
    void ok( const std::string& message="" ) { status_.ok( subsysName_, message ); };

    //! Passes this information to the system Status.
    void warning( const std::string& message ) { status_.warning( subsysName_, message ); };

    //! Passes this information to the system Status.
    void critical( const std::string& message ) { status_.critical( subsysName_, message ); };

    //! Returns subsystem's name
    std::string name() const { return subsysName_; };

    // Returns system Status object
//     Status& status() { return status_; };

private:

    Status& status_;
    std::string subsysName_;
};

} // namespace

#endif
