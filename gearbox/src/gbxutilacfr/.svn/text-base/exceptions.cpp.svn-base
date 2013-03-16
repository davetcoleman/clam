/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "exceptions.h"
#include <iostream>
#include <cstring>

using namespace std;

namespace gbxutilacfr {

Exception::Exception( const char *file, const char *line, const std::string &message )
    : message_(toMessageString(file,line,message))
{}

Exception::~Exception() throw()
{
}

std::string
Exception::toMessageString( const char *file, const char *line, const std::string &message ) const
{
    std::string msg = "*** ERROR(";
    msg += basename(file);
    msg += ":";
    msg += line;
    msg += "): " + message;
    return msg;
}

std::string basename( const std::string &s )
{
#ifndef WIN32
    size_t slashPos = s.rfind( '/' );
#else
    size_t slashPos = s.rfind( '\\' );
#endif
    if ( slashPos == s.npos )
        return s; // no slash found
    else
        return s.substr( slashPos+1, s.npos );
};

std::string dirname( const std::string &s )
{
#ifndef WIN32
    size_t slashPos = s.rfind( '/' );
#else
    size_t slashPos = s.rfind( '\\' );
#endif
    if ( slashPos == s.npos )
        return ""; // no slash found
    else
        return s.substr( 0, slashPos );
};

} // namespace
