/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <sstream>
#include "trivialstatus.h"
#include "exceptions.h"
#include <assert.h>

using namespace std;

namespace gbxutilacfr {

TrivialStatus::TrivialStatus( Tracer& tracer,
        bool stateChange, bool ok, bool warn, bool fault, bool heartbeat ) :
    tracer_(tracer),
    stateChange_(stateChange),
    ok_(ok),
    warn_(warn),
    fault_(fault),
    heartbeat_(heartbeat)
{
}

void
TrivialStatus::addSubsystem( const std::string& subsystem, double maxHeartbeatIntervalSec, SubsystemType type )
{
    tracer_.warning( "TrivialStatus: this implementation of Status does not store status of the subsystems." );
}

void
TrivialStatus::removeSubsystem( const std::string& subsystem )
{
    tracer_.warning( "TrivialStatus: this implementation of Status does not store status of the subsystems." );
}

std::vector<std::string>
TrivialStatus::subsystems()
{
    return std::vector<std::string> ();
}

SubsystemStatus
TrivialStatus::subsystemStatus( const std::string& subsystem )
{
    throw Exception( ERROR_INFO, "TrivialStatus: this implementation of Status does not store status of the subsystems." );
}

ComponentStatus
TrivialStatus::componentStatus()
{
    throw Exception( ERROR_INFO, "TrivialStatus: this implementation of Status cannot determine component status." );
}

void
TrivialStatus::setMaxHeartbeatInterval( const std::string& subsystem, double maxHeartbeatIntervalSec )
{
    // does nothing
}

void
TrivialStatus::setSubsystemType( const std::string& subsystem, SubsystemType type )
{
    // does nothing
}

void
TrivialStatus::setSubsystemStatus( const std::string& subsystem, gbxutilacfr::SubsystemState state,
        gbxutilacfr::SubsystemHealth health, const std::string& msg )
{
    internalSetStatus( subsystem, state, health, msg );
}

void
TrivialStatus::initialising( const std::string& subsystem )
{
    internalSetStatus( subsystem, gbxutilacfr::SubsystemInitialising, (gbxutilacfr::SubsystemHealth)-1 );
}

void
TrivialStatus::working( const std::string& subsystem )
{
    internalSetStatus( subsystem, gbxutilacfr::SubsystemWorking, (gbxutilacfr::SubsystemHealth)-1 );
}

void
TrivialStatus::finalising( const std::string& subsystem )
{
    internalSetStatus( subsystem, gbxutilacfr::SubsystemFinalising, (gbxutilacfr::SubsystemHealth)-1 );
}

void
TrivialStatus::fault( const std::string& subsystem, const std::string& msg )
{
    internalSetStatus( subsystem, gbxutilacfr::SubsystemFault, gbxutilacfr::SubsystemCritical, msg );
}

void
TrivialStatus::ok( const std::string& subsystem, const std::string& msg )
{
    internalSetStatus( subsystem, (gbxutilacfr::SubsystemState)-1, gbxutilacfr::SubsystemOk, msg );
}

void
TrivialStatus::warning( const std::string& subsystem, const std::string& msg )
{
    internalSetStatus( subsystem, (gbxutilacfr::SubsystemState)-1, gbxutilacfr::SubsystemWarning, msg );
}

void
TrivialStatus::critical( const std::string& subsystem, const std::string& msg )
{
    internalSetStatus( subsystem, (gbxutilacfr::SubsystemState)-1, gbxutilacfr::SubsystemCritical, msg );
}

void
TrivialStatus::heartbeat( const std::string& subsystem )
{
    internalSetStatus( subsystem, (gbxutilacfr::SubsystemState)-1, (gbxutilacfr::SubsystemHealth)-1 );
}

void
TrivialStatus::message( const std::string& subsystem, const std::string& msg )
{
    internalSetStatus( subsystem, (gbxutilacfr::SubsystemState)-1, (gbxutilacfr::SubsystemHealth)-1, msg );
}

void
TrivialStatus::internalSetStatus( const std::string& subsystemName, gbxutilacfr::SubsystemState state,
                gbxutilacfr::SubsystemHealth health, bool hasMessage, const std::string& msg )
{
//     cout<<"DEBUG: state="<<state<<" health="<<health<<" msg="<<msg<<endl;
    assert( state!=gbxutilacfr::SubsystemIdle && "Idle state should not be reported from within the subsystem" );

    // if this is a heartbeat, do nothing else
    if ( state<0 && health<0 && !hasMessage )
        return;
    bool traceState = true;
    // don't trace if we don't know it
    if ( state < 0 )
        traceState = false;

    bool traceHealth = true;
    // don't trace if we don't know it
    if ( health < 0 )
        traceHealth = false;

//     bool publishStatus = false;

    // give tracer feedback on state
    if ( traceState )
        tracer_.info( "Subsystem '"+subsystemName+"' changed state to "+gbxutilacfr::toString(state) );

    // give tracer feedback on status
    if ( traceHealth )
    {
        // ugly stuff because we don't remember our state.
        string stateString = (state<0) ? "Unchanged" : gbxutilacfr::toString(state);
        string trace = "Subsystem '"+subsystemName
                +"' status="+stateString+"/"+gbxutilacfr::toString(health);

        if ( !msg.empty() )
            trace =+ ": "+msg;
        switch ( health )
        {
        case gbxutilacfr::SubsystemOk :
            tracer_.info( trace );
            break;
        case gbxutilacfr::SubsystemWarning :
            tracer_.warning( trace );
            break;
        case gbxutilacfr::SubsystemCritical :
            tracer_.error( trace );
            break;
        }
    }

    // publish updated status
//     if ( publishStatus )
//         localPublish();
}

void
TrivialStatus::process()
{
    // cannot determine stalls because we are not tracking subsystem status
}

}
