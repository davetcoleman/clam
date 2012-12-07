/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_SMARTBATTERYACFR_EXCEPTIONS_H
#define GBX_SMARTBATTERYACFR_EXCEPTIONS_H

#include <gbxutilacfr/exceptions.h>

namespace gbxsmartbatteryacfr {
    
//! Exception for hardware reading problems
class HardwareReadingException : public gbxutilacfr::Exception
{  
    public:
        HardwareReadingException( const char *file, const char *line, const char *message )
        : Exception( file, line, message ) {}     
};

//! Exception for parsing problems
class ParsingException : public gbxutilacfr::Exception
{  
    public:
        ParsingException( const char *file, const char *line, const char *message )
        : Exception( file, line, message ) {}    
};

}

#endif

