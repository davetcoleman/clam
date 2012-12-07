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
#include <cstdlib>
#include <gbxutilacfr/trivialtracer.h>
#include <gbxutilacfr/trivialstatus.h>
#include <gbxutilacfr/exceptions.h>

using namespace std;

int main(int argc, char * argv[])
{
    gbxutilacfr::TrivialTracer tracer;

    gbxutilacfr::TrivialStatus status( tracer );

    cout<<"setMaxHeartbeatInterval on non-existent subsystems ...";
    try {
        status.setMaxHeartbeatInterval( "core", 10 );
    }
    catch ( const gbxutilacfr::Exception& e ) {
        // ok
    }
    cout<<"ok"<<endl;

    cout<<"adding subsystem ...";
    status.addSubsystem( "core", 10 );

    // not calling subsystems(), it throws

    // not calling subsystemStatus(), it throws

    status.setMaxHeartbeatInterval( "core", 20 );
    status.initialising( "core" );
    status.ok( "core", "holding fingers crossed" );

    // this one should not appear
    status.ok( "core", "holding fingers crossed" );
    status.initialising( "core" );
    status.ok( "core", "still holding fingers crossed" );
    status.heartbeat( "core" );

    status.warning( "core", "all is weird" );
    status.warning( "core", "all is weird" );
    status.warning( "core", "still all is weird" );

    status.critical( "core", "all is bad" );
    status.critical( "core", "all is bad" );
    status.critical( "core", "still all is bad" );

    status.ok( "core", "all is good again" );
    status.ok( "core" );
    // this one will not get traced!
    status.ok( "core" );

    status.working( "core" );

    status.finalising( "core" );

    // should delete and start complaining again
    cout<<"removing existing subsystem ...";
    status.removeSubsystem( "core" );
    cout<<"ok"<<endl;

    return EXIT_SUCCESS;
}
