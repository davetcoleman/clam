/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "tokenise.h"
#include <iostream>

using namespace std;

namespace gbxutilacfr {
    
std::vector<string> tokenise( const string &str, const string &delimiter )
{
#define SKIP_DELIMS 0
#if !SKIP_DELIMS
    std::vector<std::string> tokens;

    string::size_type lastPos = 0;
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiter, lastPos);

    while ( pos != string::npos )
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = pos+1;

        // Find first "non-delimiter".
        pos     = str.find_first_of(delimiter, lastPos);
    }
    tokens.push_back(str.substr(lastPos,str.size()));

    return tokens;


#else

    std::vector<std::string> tokens;

    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiter, 0);
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiter, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));

        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiter, pos);

        // Find next "non-delimiter"
        pos = str.find_first_of(delimiter, lastPos);
    }

    return tokens;
#endif
}

}
