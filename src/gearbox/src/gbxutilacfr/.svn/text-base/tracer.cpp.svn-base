/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "tracer.h"
#include <assert.h>

namespace gbxutilacfr {

std::string toString( TraceType type )
{
    switch ( type ) 
    {
    case InfoTrace :
        return "info";
    case WarningTrace :
        return "warn";
    case ErrorTrace :
        return "error";
    case DebugTrace :
        return "debug";
    case AnyTrace :
        return "any";
    case NumberOfTraceTypes :
        return "number_of_trace_types";
    default:
        assert( !"gbxutilacfr::Tracer::toString() should never get to default" );
    }
}

} // namespace
