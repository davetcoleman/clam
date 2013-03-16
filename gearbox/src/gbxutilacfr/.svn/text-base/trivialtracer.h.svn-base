/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_TRIVIAL_TRACER_H
#define GBXUTILACFR_TRIVIAL_TRACER_H

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

#include <gbxutilacfr/tracer.h>

namespace gbxutilacfr {

//!
//! @brief A simple implementation of the tracer API which prints to cout.
//!
//! @see Tracer
//!
class GBXUTILACFR_EXPORT TrivialTracer : public Tracer
{
public:

    //!
    //! Integers specify the tracing level: we'll print out all level at or below this.
    //!
    TrivialTracer( int debug=0,
                   int info=9,
                   int warn=9,
                   int error=9 );

    virtual void print( const std::string &message );
    virtual void info( const std::string &message, int level=1, bool localOnly=false )
        { info("",message,level,localOnly); }
    virtual void warning( const std::string &message, int level=1, bool localOnly=false )
        { warning("",message,level,localOnly); }
    virtual void error( const std::string &message, int level=1, bool localOnly=false )
        { error("",message,level,localOnly); }
    virtual void debug( const std::string &message, int level=1, bool localOnly=false )
        { debug("",message,level,localOnly); }
    virtual int verbosity( TraceType traceType, DestinationType destType=ToAny ) const;

    virtual void info( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false );
    virtual void warning( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false );
    virtual void error( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false );
    virtual void debug( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false );

private:

    int traceLevels_[NumberOfTraceTypes];
};

} // namespace

#endif
