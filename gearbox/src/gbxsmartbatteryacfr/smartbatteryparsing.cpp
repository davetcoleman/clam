/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <sstream>
#include <gbxsmartbatteryacfr/exceptions.h>

#include "smartbatteryparsing.h"

using namespace std;

namespace gbxsmartbatteryacfr {

namespace {
    
int readNumPositiveFlags(const string &str)
{
    vector<bool> flags;
    readFlags( str, flags );
    
    int numPositiveFlags=0;

    for (unsigned int i=0; i<flags.size(); i++)
    {
        if (flags[i]==true)
            numPositiveFlags++;
    }
     
    return numPositiveFlags;
}

int readUnsignedInt( const string &str )
{
    stringstream ss(str);
    int value;
    ss >> std::hex >> value;
    return value;
}

int readUnsignedInt16( const string &str )
{    
    if (str.size()!=4) 
        throw ParsingException( ERROR_INFO, "readUnsignedInt16 called with string size != 4" );
    
    // There's a known bug in the Oceanserver system which sometimes produces an FFFF record
    // According to Kevin Ludlam (ludlam@ocean-server.com), "caused by a collision on the SMBus"
    if (str=="FFFF")
        throw ParsingException( ERROR_INFO, "Read FFFF as a value. May be caused by a collision on the SMBus" );

    return readUnsignedInt( str );
}

int readUnsignedInt8( const string &str )
{    
    if (str.size()!=2) 
        throw ParsingException( ERROR_INFO, "readUnsignedInt8 called with string size != 2" );
    
    // There's a known bug in the Oceanserver system which sometimes produces an FF record
    // According to Kevin Ludlam (ludlam@ocean-server.com), "caused by a collision on the SMBus"
    if (str=="FF")
        throw ParsingException( ERROR_INFO, "Read FF as a value. May be caused by a collision on the SMBus" );

    return readUnsignedInt( str );
}

int16_t readSignedInt16( const string &str )
{
    if (str.size()!=4) 
        throw ParsingException( ERROR_INFO, "readSignedInt16 called with string size != 4" );
    
    // There's a known bug in the Oceanserver system which sometimes produces an FFFF record
    // According to Kevin Ludlam (ludlam@ocean-server.com), "caused by a collision on the SMBus"
    if (str=="FFFF")
        throw ParsingException( ERROR_INFO, "Read FFFF as a value. May be caused by a collision on the SMBus" );
   
    stringstream ss(str);
    int value;
    ss >> std::hex >> value;
    int16_t value16 = value;

    return value16;
}

} // end of namespace


double readTemperature( const string &str )
{
    const double kelvin2Celsius = -272.15;
    return ( (double)readUnsignedInt16( str )/10.0 + kelvin2Celsius );
}

double readCurrent( const string &str )
{   
    return (double)readSignedInt16( str )/1000.0;
}

double readVoltage( const string &str )
{
    return (double)readUnsignedInt16( str )/1000.0;
}

int readNumBatteries( const string &str )
{
    return readNumPositiveFlags( str );
}

int readPercentWord( const string &str )
{
    return readUnsignedInt16( str );    
}

int readPercentByte( const string &str )
{
    return readUnsignedInt8( str );    
}

int readMinutes( const string &str )
{
    return readUnsignedInt16( str );   
}

int readCapacity( const string &str )
{
    return readUnsignedInt16( str );       
}

uint16_t read16Flags( const string &str )
{
    int flags = readUnsignedInt16( str );
    uint16_t uFlags = flags;
    return uFlags;
}

int readCount( const string &str )
{
    return readUnsignedInt16( str );   
}

int readNumber( const string &str )
{
    return readUnsignedInt16( str );   
}

int readRate( const string &str )
{
    return readUnsignedInt16( str );
}

bool isChecksumValid( const string &data, 
                      const string &expectedChecksumStr )
{ 
    int computedChecksum=0;
    
    // skip the first character which is '$'
    for (unsigned int i=1; i<data.size(); i++)
    {
        //bytewise XOR
        computedChecksum ^= data[i];
    }

    stringstream ss(expectedChecksumStr);
    int expectedChecksum;
    ss >> std::hex >> expectedChecksum;
       
    if (expectedChecksum==computedChecksum)
        return true;

    return false;
}

void toKeyValuePairs( const vector<string> &fields,
                      map<string,string>   &pairs,
                      gbxutilacfr::Tracer  &tracer)
{
    stringstream ss;
    ss << "SmartBatteryParsing: fields size: " << fields.size() << endl;
    ss << "SmartBatteryParsing: mod operation (expect 0): " << fields.size()%2;
    tracer.debug( ss.str(), 10 );
    
    // make sure we have an equal number
    if (fields.size()%2 != 0)
        throw ParsingException( ERROR_INFO, "toKeyValuePairs: odd number of inputs" );
    
    unsigned int i=0;
    while(true)
    {   
        if (fields.size() <= i+1)
            throw ParsingException( ERROR_INFO, "toKeyValuePairs: wrong number of inputs" );
        pairs[fields[i]] = fields[i+1];
        i=i+2;
        if (fields.size()==i) break;
    }
    
    ss.str(""); ss << "SmartBatteryParsing: pairs size: " << pairs.size();
    tracer.debug( ss.str(), 10 );

}


void readFlags( const string &str, 
                vector<bool> &flags )
{
    if (str.size()!=2) 
        throw ParsingException( ERROR_INFO, "readFlags called with string size != 2" );
    
    stringstream ss(str);
    int allFlags;
    ss >> std::hex >> allFlags;
    
    flags.resize(8);
    
    int mask = 0x01;
    for ( int i=0; i < 8; i++ )
    {
        flags[i] = allFlags & mask;
        mask = mask << 1;
    }    
}

}
