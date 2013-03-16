/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "status.h"
#include <sstream>
#include <assert.h>

namespace gbxutilacfr {

std::string toString( SubsystemState state )
{
    switch ( state )
    {
    case SubsystemInitialising :
        return "Initialising";
    case SubsystemWorking :
        return "Working";
    case SubsystemFinalising :
        return "Finalising";
    case SubsystemIdle :
        return "Idle";
    case SubsystemShutdown :
        return "Shutdown";
    case SubsystemFault :
        return "Fault";
    }
	return ""; // Shuts up MSVC
}

std::string toString( SubsystemHealth health )
{
    switch ( health )
    {
    case SubsystemOk :
        return "Ok";
    case SubsystemWarning :
        return "Warning";
    case SubsystemCritical :
        return "Critical";
    }
	return ""; // Shuts up MSVC
}

std::string toString( const SubsystemStatus& status )
{
    std::stringstream ss;
    ss << "state="<<toString(status.state)
       << " health="<<toString(status.health)
       << " msg='"<<status.message<<"'"
       << " since hearbeat="<<status.sinceHeartbeat;
    return ss.str();
	return ""; // Shuts up MSVC
}

std::string toString( SubsystemType type )
{
    switch ( type )
    {
    case SubsystemStandard :
        return "Standard";
    case SubsystemEarlyExit :
        return "EarlyExit";
    case SubsystemInfrastructure :
        return "Infrastructure";
    }
	return ""; // Shuts up MSVC
}

std::string toString( ComponentState state )
{
    switch ( state )
    {
    case CompStarting :
        return "Starting";
    case CompOperational :
        return "Operational";
    case CompStopping :
        return "Stopping";
    case CompFault :
        return "Fault";
    }
    return ""; // Shuts up MSVC
}

std::string toString( ComponentHealth health )
{
    switch ( health )
    {
    case CompOk :
        return "Ok";
    case CompWarning :
        return "Warning";
    case CompCritical :
        return "Critical";
    }
    return ""; // Shuts up MSVC
}

} // namespace
