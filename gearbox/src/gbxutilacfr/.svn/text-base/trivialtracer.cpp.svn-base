/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "trivialtracer.h"
#include <iostream>
#include <assert.h>

using namespace std;

namespace gbxutilacfr {

TrivialTracer::TrivialTracer( int debug, int info, int warn, int error )
{
    traceLevels_[InfoTrace]    = info;
    traceLevels_[WarningTrace] = warn;
    traceLevels_[ErrorTrace]   = error;
    traceLevels_[DebugTrace]   = debug;
}

void
TrivialTracer::print( const std::string &message )
{
    cout<<message<<endl;
}

void
TrivialTracer::info( const std::string &subsystem, const std::string &message, int level, bool localOnly )
{
    if ( traceLevels_[InfoTrace] >= level )
    {
        cout << "info: ";
        if ( subsystem != "" )
            cout << subsystem << ": ";
        cout << message << endl;
    }
}

void
TrivialTracer::warning( const std::string &subsystem, const std::string &message, int level, bool localOnly )
{
    if ( traceLevels_[WarningTrace] >= level )
    {
        cout << "warn: ";
        if ( subsystem != "" )
            cout << subsystem << ": ";
        cout << message << endl;
    }
}
    
void
TrivialTracer::error( const std::string &subsystem, const std::string &message, int level, bool localOnly )
{
    if ( traceLevels_[ErrorTrace] >= level )
    {
        cout << "error: ";
        if ( subsystem != "" )
            cout << subsystem << ": ";
        cout << message << endl;
    }
}

void
TrivialTracer::debug( const std::string &subsystem, const std::string &message, int level, bool localOnly )
{
    if ( traceLevels_[DebugTrace] >= level )
    {
        cout << "debug: ";
        if ( subsystem != "" )
            cout << subsystem << ": ";
        cout << message << endl;
    }
}

int 
TrivialTracer::verbosity( TraceType traceType, DestinationType destType ) const
{
    assert( traceType >= 0 && traceType <= NumberOfTraceTypes );
    return traceLevels_[traceType];
}

}
