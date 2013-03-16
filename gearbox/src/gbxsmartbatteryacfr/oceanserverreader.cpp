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
#include <cstring>
#include <gbxsmartbatteryacfr/exceptions.h>

#include "oceanserverreader.h"

using namespace std;

namespace gbxsmartbatteryacfr {
    
namespace { 
       
    // baudrate is fixed
    static const int BAUDRATE = 19200;
    
    // timeout for reading from the serial port
    static const int TIMEOUT_SEC = 2;
    
    // the maximum number of lines to read until we are confident that we are not
    // connected to an OceanServer system
    static const int MAX_TRIES = 50;
    
    // Returns true if 100% sure that we are connected to an OceanServer system, otherwise false
    bool isOceanServerSystem( const string &candidateString )
    {   
        // number of characters which we require to match
        // if they match, we are pretty sure we have an OceanServerSystem
        unsigned int numCharRequired = 8;
        if ( candidateString.size()<numCharRequired ) return false;
    
        // some menu entries from the OceanServer system - used to recognize whether
        // we are connected to the right device
        std::vector<std::string> oceanServerStrings;
        oceanServerStrings.push_back(" S - Setup Controller");
        oceanServerStrings.push_back(" B - Battery Status");
        oceanServerStrings.push_back(" X - Host HEX");
        oceanServerStrings.push_back(" H - Help");
        oceanServerStrings.push_back(" www.ocean-server.com");
        
        for (unsigned int i=0; i<oceanServerStrings.size(); i++) 
        {
            if ( strncmp(oceanServerStrings[i].c_str(),candidateString.c_str(),numCharRequired)==0 ) 
                return true;
        }
        return false;
    }

}

 
OceanServerReader::OceanServerReader( const string        &serialPort,
                                      gbxutilacfr::Tracer &tracer )
    : serial_( serialPort, BAUDRATE, gbxserialacfr::Serial::Timeout(TIMEOUT_SEC,0) ),
      tracer_(tracer),
      parser_(tracer),
      firstTime_(true)
{
    checkConnection();
    reset();
}

void
OceanServerReader::reset()
{
    serial_.flush();

    // send the command to get into menu mode
    const char menuMode = ' ';
    serial_.write(&menuMode, 1);

    // send the command to start sending hex data
    const char startSendingHex = 'X';
    serial_.write(&startSendingHex, 1);

    firstTime_ = true;
}

void
OceanServerReader::checkConnection()
{   
    tracer_.debug( "OceanServerReader: Checking connection to serial port", 3 );

    // a blank will get us into menu mode
    const char menuMode = ' ';
    serial_.write(&menuMode, 1);
    
    int numTries=0;
    stringstream ss;

    while(true)
    {
        ss.str(""); ss << "OceanServerReader: " << __func__ << ": Trying to read from serial port with timeout of " << TIMEOUT_SEC << "s" << endl;
        tracer_.debug( ss.str(), 3 );
        
        // reading from serial port
        string serialData;        
        int ret = serial_.readLine( serialData );
        if ( ret<0 )  {
          throw HardwareReadingException( ERROR_INFO, "Timed out while trying to read a line from serial port.");
        }
        
        // checking whether we are connected to an oceanserver system
        if ( isOceanServerSystem(serialData) ) {
            tracer_.debug( "OceanServerReader: We are connected to an Oceanserver system. Good.", 3 );
            break;
        }
        
        // count the number of tries
        numTries++;
        ss.str(""); ss << "OceanServerReader: Trying to find out whether this is an oceanserver system. Attempt number " << numTries << "/" << MAX_TRIES << ".";
        tracer_.debug( ss.str(), 3 );
        
        if ( numTries >= MAX_TRIES ) {
            throw HardwareReadingException( ERROR_INFO, "Didn't recognize any of the strings. We may be connected to the wrong serial port.");
        }
    }
}    

std::string
OceanServerReader::tryToReadLineFromSerialPort()
{
    const int maxTries=5;
    int numTries=0;
    string serialData;
    
    while(true)
    {
        int ret = serial_.readLine( serialData );

        if (ret>0) break;
        
        numTries++;
        if (numTries>=maxTries) {
            stringstream ss;
            ss << "Can't read data from serial port. Timed out and/or empty strings " << maxTries << " times in a row.";
            throw HardwareReadingException( ERROR_INFO,  ss.str().c_str() );
        }
    }
    
    return serialData;
}

void 
OceanServerReader::read( OceanServerSystem &system )
{
    string serialData;
    vector<string> stringList;

    if (firstTime_) 
    {
        // (1) Wait until we got the beginning of the record
        int numTries=0;
        while(true)
        {
            if ( numTries > MAX_TRIES ) 
                throw gbxutilacfr::Exception( ERROR_INFO, "Couldn't find the beginning of a valid OceanServer record" );
            numTries++;
            serialData = tryToReadLineFromSerialPort();
            if (parser_.atBeginningOfRecord( serialData )) break;
        }
        
        tracer_.debug( "OceanServerReader: Beginning of a new record", 5 );
        
        // (2) Add the first line to the stringlist
        stringList.push_back( serialData );
    }
    else
    {
        tracer_.debug( "OceanServerReader: We already have the first line from the previous record", 5 );
        stringList.push_back( beginningRecordLine_ );
    }
    
    try {
        
        // (3) Read the rest of the record line-by-line
        int numTries=0;
        while(true)
        {        
            if ( numTries > MAX_TRIES ) 
                throw gbxutilacfr::Exception( ERROR_INFO, "Couldn't find the beginning of a SUBSEQUENT valid OceanServer record" );
            numTries++;
            serialData = tryToReadLineFromSerialPort();
            if ( parser_.atBeginningOfRecord( serialData ) ) 
            {   
                tracer_.debug( "OceanServerReader: End of the record (beginning of a SUBSEQUENT record)", 5 );
                parser_.parse( stringList, system );
                break; 
            }
            stringList.push_back(serialData);
        }
        
        // (4) Save the beginning of the next record
        beginningRecordLine_ = serialData;
        if (firstTime_) {
            firstTime_ = false;
        }
    } 
    catch (ParsingException &e)
    {
        stringstream ss;
        ss << "OceanServerReader: Caught ParsingException: " << e.what() << ". ";
        ss << "It's not critical, we will try to find the beginning of a new record.";
        tracer_.debug( ss.str(), 3 );
        firstTime_ = true;
        
        // we have to rethrow, so that the caller knows that it may receive a corrupt record
        throw;
    }
}

}
         
