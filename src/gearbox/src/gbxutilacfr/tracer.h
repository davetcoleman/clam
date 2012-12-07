/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://!gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_TRACER_H
#define GBXUTILACFR_TRACER_H

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

#include <string>

namespace gbxutilacfr {

//! Types of traced information
enum TraceType {
    //! Information
    InfoTrace=0,
    //! Warning
    WarningTrace,
    //! Error
    ErrorTrace,
    //! Debug statement
    DebugTrace,
    //! Use this index to find out the maximum verbosity among all trace types to
    //! a particular destination.
    AnyTrace,
    //! Number of trace types
    NumberOfTraceTypes,
};

//! Returns a string corresponding to the enum element.
GBXUTILACFR_EXPORT std::string toString( TraceType type );

//! Types of destinations for traced information.
enum DestinationType {
    //! Write to stardard display
    ToDisplay=0,
    //! Send over the network, details are specific to Tracer implementation
    ToNetwork,
    //! Write to SysLog on Unix, EventLog on windows
    ToLog,
    //! Write to a file
    ToFile,
    //! Use this index to request the maximum verbosity of a particular type among 
    //! all destinations
    ToAny,
    //! Number of destination types
    NumberOfDestinationTypes
};

//! 
//! @brief Local and remote tracing.
//! 
//! Tracer is used to log human-readable trace statements, e.g. warnings, error messages, etc (not data).
//! 
//! A single Tracer object is meant to be shared by multiple threads, so the
//! implementation must be thread-safe.
//! 
//! When the tracing message is cheap to generate, simply call one of the tracing functions.
//! The routing to destinations will be performed internally.
//! 
//! @verbatim
//! std::string s = "something is wrong";
//! tracer->error( s );
//! @endverbatim
//! 
//! If the tracing message is a result of an expensive operation, you
//! many want to perform the tracing test yourself and then call the
//! tracing function. You may test verbosity for specific TraceType /
//! DestinationType combinations or use summary fields AnyTrace and
//! ToAny.
//! 
//! @verbatim
//! Tracer* tracer = context().tracer();
//! if ( tracer.verbosity( gbxutilacfr::Tracer::ErrorTrace, gbxutilacfr::Tracer::ToAny ) > 0 ) {
//!     std::string s = expensiveOperation();
//!     tracer.error( s );
//! }
//! @endverbatim
//! 
//! Enum gbxutilacfr::TraceType defines types of traced
//! information. Enum gbxutilacfr::DestinationType defines possible
//! tracer destinations are. Verbosity levels range from 0 (nothing) to
//! 10 (everything). The built-in defaults are as follows:
//! @verbatim
//!                 ToDisplay   ToNetwork   ToLog   ToFile
//! Info                1           0         0       0
//! Warning             1           0         0       0
//! Error              10           0         0       0
//! Debug               0           0         0       0
//! @endverbatim
//! 
//! @see Status
//!
class GBXUTILACFR_EXPORT Tracer
{
public:
    virtual ~Tracer() {}; 

    struct Config
    {
        //! array of verbosity levels
        int verbosity[NumberOfTraceTypes][NumberOfDestinationTypes];
        //! affects only the printout to stdout. Remote messages always have a timestamp.
        bool addTimestamp;
    };

    //! LOCAL INTERFACE
    
    //! Prints out verbatim to stdout. It is never routed over the network.
    //! @see info
    virtual void print( const std::string &message ) = 0;

    //! Routing is determined by InfoToXxx parameter.
    //! If localOnly is set to TRUE, messages are not sent over the network (useful when traces
    //! is caused by network errors).
    virtual void info( const std::string &message, int level=1, bool localOnly=false )
        { info("",message,level,localOnly); }
    //! This version gives the Tracer some hints about a specific subsytem which generated the message
    virtual void info( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false ) = 0;
    
    //! Routing is determined by WarningToXxx parameter.
    //! If localOnly is set to TRUE, messages are not sent over the network (useful when traces
    //! is caused by network errors).
    virtual void warning( const std::string &message, int level=1, bool localOnly=false )
        { warning("",message,level,localOnly); }
    //! This version gives the Tracer some hints about a specific subsytem which generated the message
    virtual void warning( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false ) = 0;
    
    //! Routing is determined by ErrorToXxx parameter.
    //! If localOnly is set to TRUE, messages are not sent over the network (useful when traces
    //! is caused by network errors).
    virtual void error( const std::string &message, int level=1, bool localOnly=false )
        { error("",message,level,localOnly); }
    //! This version gives the Tracer some hints about a specific subsytem which generated the message
    virtual void error( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false ) = 0;

    //! Routing is determined by DebugToXxx parameter.
    //! If localOnly is set to TRUE, messages are not sent over the network (useful when traces
    //! is caused by network errors).
    virtual void debug( const std::string &message, int level=1, bool localOnly=false )
        { debug("",message,level,localOnly); }
    //! This version gives the Tracer some hints about a specific subsytem which generated the message
    virtual void debug( const std::string &subsystem, const std::string &message, int level=1, bool localOnly=false ) = 0;

    //! Returns the verbosity level for traceType to destType. This test is performed 
    //! internally by all tracing functions, e.g. error(). You may want to call this 
    //! function yourself @e before calling error() if there is a significant overhead
    //! in forming the tracing string. See class documentation for an example of such
    //! usage.
    virtual int verbosity( TraceType traceType, DestinationType destType=ToAny ) const = 0;

    //! @b EXPERIMENTAL. Sets debug level for an individual subsystem.
    //! Only debug-to-display is supported.
    virtual void setSubsystemDebugLevel( const std::string &subsystem, int level=0 ) {};

    //! @b EXPERIMENTAL. Debug tracing for an individual subsystem.
    //! Only debug-to-display is supported.
    //! The maximum traceable level is set per-subsystem with setLevel().
    virtual void subsystemDebug( const std::string &subsystem, const std::string &message, int level=1 ) 
    {
        debug( message, level );
    };
};

} //! namespace

#endif
