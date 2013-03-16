#include "printutil.h"
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

namespace gbxserialdeviceacfr {

//////////////////////////////////////////////////////////////////////
// Printing Functions
//////////////////////////////////////////////////////////////////////

std::string 
toHexString( const char *buf, int bufLen )
{
    stringstream ss;
    ss << "[ ";
    for ( int i=0; i < bufLen; i++ )
    {
        ss <<hex<<std::setfill('0')<<std::setw(2)<<(int)((unsigned char)buf[i])<<" ";
    }
    ss << "]";
    return ss.str();
}

std::string 
toAsciiString( const char *buf, int bufLen )
{
    stringstream ss;
    ss << "[ ";
    for ( int i=0; i < bufLen; i++ )
        ss <<buf[i];
    ss << " ]";
    return ss.str();
}

}

