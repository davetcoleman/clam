/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_TOKENISE_H
#define GBXUTILACFR_TOKENISE_H

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
#include <vector>

namespace gbxutilacfr {

//! Takes a string containing tokens separated by a delimiter
//! Returns the vector of tokens
GBXUTILACFR_EXPORT std::vector<std::string> tokenise( const std::string &str, 
                                                      const std::string &delimiter );

}

#endif
