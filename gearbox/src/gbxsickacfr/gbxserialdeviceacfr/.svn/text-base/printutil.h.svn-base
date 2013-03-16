#ifndef GBXSERIALDEVICEACFR_PRINTUTIL_H
#define GBXSERIALDEVICEACFR_PRINTUTIL_H

#include <vector>
#include <string>

namespace gbxserialdeviceacfr {

//////////////////////////////////////////////////////////////////////
// Printing Functions
//////////////////////////////////////////////////////////////////////

// For printing binary data
std::string toHexString( const char *buf, int bufLen );
inline std::string toHexString( const std::vector<char> &buf )
{return toHexString( &(buf[0]), buf.size() );}
inline std::string toHexString( const std::string &buf )
{return toHexString( &(buf[0]), buf.size() );}

// For printing ascii data
std::string toAsciiString( const char *buf, int bufLen );
inline std::string toAsciiString( const std::vector<char> &buf )
{return toAsciiString( &(buf[0]), buf.size() );}

}

#endif
