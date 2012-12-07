/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include "serialdevicehandler.h"
#include <iostream>
#include <cstring>
#include "printutil.h"

using namespace std;

namespace gbxserialdeviceacfr {

namespace {

    // Clear out the bytes we've just parsed, and shift the un-parsed data
    // to the front of the buffer.
    void removeParsedData( std::vector<char> &buffer,
                           int numBytesParsed )
    {
        if ( numBytesParsed > (int)(buffer.size()) )
        {
            stringstream ss;
            ss << "Huh? numBytesParsed("<<numBytesParsed<<") is bigger than buffer.size()("<<buffer.size()<<")";
            throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
        }

        if ( numBytesParsed > 0 )
        {
            int numBytesBeyond = buffer.size()-numBytesParsed;
            if ( numBytesBeyond > 0 )
            {
                memmove( &(buffer[0]), &(buffer[numBytesParsed]), numBytesBeyond*sizeof(char) );
            }
            assert( numBytesBeyond >= 0 );
            buffer.resize( numBytesBeyond );
        }
    }

}

//////////////////////////////////////////////////////////////////////

SerialDeviceHandler::SerialDeviceHandler( const std::string     &subsysName,
                                          gbxserialacfr::Serial &serialPort,
                                          RxMsgParser           &rxMsgParser,
                                          gbxutilacfr::Tracer   &tracer,
                                          gbxutilacfr::Status   &status,
                                          bool                   verboseDebug,
                                          int                    unparsedBytesWarnThreshold )
    : gbxiceutilacfr::SafeThread( tracer ),
      serial_(serialPort),
      rxMsgParser_(rxMsgParser),
      rxMsgCallback_(NULL),
      unparsedBytesWarnThreshold_(unparsedBytesWarnThreshold),
      tracer_(tracer),
      subStatus_( status, subsysName ),
      verboseDebug_(verboseDebug)
{
}

SerialDeviceHandler::~SerialDeviceHandler()
{
    //
    // The component may outlive this subsystem.
    // So tell status that it might not hear from us for a while.
    //
    subStatus_.setMaxHeartbeatInterval( -1 );
}

void
SerialDeviceHandler::setBaudRate( int baudRate )
{
    tracer_.debug( "SerialDeviceHandler: Changing baud rate of serial port." );
    serial_.setBaudRate( baudRate );
    // TODO: AlexB: not entirely sure if these are necessary...
    serial_.flush();
    serial_.drain();
}

void
SerialDeviceHandler::send( const char *commandBytes, int numCommandBytes )
{
    if ( verboseDebug_ )
    {
        stringstream ss;
        ss << "SerialDeviceHandler::"<<__func__<<"(): Sending "<<numCommandBytes<<" bytes";
        tracer_.debug( ss.str(), 5 );
    }
    serial_.write( commandBytes, numCommandBytes );
}

void
SerialDeviceHandler::walk()
{
    subStatus_.working();
    double maxIntervalSec = serial_.timeout().sec + 1e6*serial_.timeout().usec;
    subStatus_.setMaxHeartbeatInterval( maxIntervalSec * 5.0 );

    isRunningStore_.set( true );

    while ( !isStopping() )
    {
        if ( verboseDebug_ )
            tracer_.debug( "SerialDeviceHandler::walk() start of loop.", 6 );

        try {

            // Wait for data to arrive
            try {
                if ( getDataFromSerial() )
                {
                    // Process it
                    try {
                        bool statusOK = processBuffer();
                        if ( statusOK )
                            subStatus_.ok();
                    }
                    catch ( std::exception &e )
                    {
                        stringstream ss;
                        ss << "SerialDeviceHandler: During processBuffer: " << e.what();
                        tracer_.error( ss.str() );
                        throw;
                    }
                }
                else
                {
                    subStatus_.ok();
                }
            }
            catch ( std::exception &e )
            {
                stringstream ss;
                ss << "SerialDeviceHandler: while reading from serial: " << e.what();
                tracer_.error( ss.str() );
                throw;
            }
        }
        catch ( IceUtil::Exception &e )
        {
            stringstream ss;
            ss << "SerialDeviceHandler: Caught Ice exception: " << e;
            subStatus_.critical( ss.str() );
        }
        catch ( std::exception &e )
        {
            stringstream ss;
            ss << "SerialDeviceHandler: Caught exception: " << e.what();
            subStatus_.critical( ss.str() );
        }
        catch ( ... )
        {
            stringstream ss;
            ss << "SerialDeviceHandler: Caught unknown exception.";
            subStatus_.critical( ss.str() );
        }
    }

    isRunningStore_.set( false );
}

void
SerialDeviceHandler::startAndWaitTillIsRunning()
{
    assert( rxMsgCallback_ );
    start();

    int numSleeps=0;
    while ( !isRunning() )
    {
        const int TEN_MS = 10000;
        usleep( TEN_MS );

        const int TEN_SEC_IN_SLEEPS = 1000;
        if ( numSleeps > TEN_SEC_IN_SLEEPS )
        {
            throw gbxutilacfr::Exception( ERROR_INFO, "Still waiting for SerialDeviceHandler thread to start after 10sec." );
        }
    }
}

bool
SerialDeviceHandler::isRunning()
{
    if ( isRunningStore_.isEmpty() )
        return false;

    bool running;
    isRunningStore_.get( running );
    return running;
}

bool
SerialDeviceHandler::getDataFromSerial()
{
    int nBytes = serial_.bytesAvailableWait();

    if ( verboseDebug_ )
    {
        stringstream ss;
        ss << "SerialDeviceHandler::getDataFromSerial(): nBytes available: " << nBytes;
        tracer_.debug( ss.str(), 9 );
    }

    if ( nBytes > 0 )
    {
        // Resize buffer to make room for new data
        buffer_.resize( buffer_.size() + nBytes );
        int readStart = buffer_.size()-nBytes;
        int numRead = serial_.read( &(buffer_[readStart]), nBytes );
        assert( (numRead == nBytes) && "serial_.read should read exactly the number we ask for." );
        
        if ( (int)(buffer_.size()) > unparsedBytesWarnThreshold_ )
        {
            stringstream ss;
            ss << "SerialDeviceHandler:: Buffer is getting pretty big -- size is " << buffer_.size();
            tracer_.warning( ss.str() );
        }

        return true;
    }
    return false;
}

bool
SerialDeviceHandler::processBuffer()
{
    if ( verboseDebug_ )
    {
        stringstream ssDebug;
        ssDebug << "SerialDeviceHandler::processSerialBuffer: buffer is: " << toAsciiString(buffer_);
        tracer_.debug( ssDebug.str() );
    }

    bool statusOK = true;

    // This loop is in case multiple messages arrived.
    while ( true )
    {
        IceUtil::Time t = IceUtil::Time::now();
        int timeStampSec = (int)t.toSeconds();
        int timeStampUsec = (int)t.toMicroSeconds() - timeStampSec*1000000;        

        RxMsgPtr rxMsg;
        int numBytesParsed = 0;
        try {
            rxMsg = rxMsgParser_.parseBuffer( buffer_,
                                              numBytesParsed );
        }
        catch ( const IceUtil::Exception &e )
        {
            stringstream ss;
            ss << "SerialDeviceHandler: While parsing buffer for rxMsgs: " << e;
            tracer_.warning( ss.str() );
            throw;
        }
        catch ( const std::exception &e )
        {
            stringstream ss;
            ss << "SerialDeviceHandler: While parsing buffer for rxMsgs: " << e.what();
            tracer_.warning( ss.str() );
            throw;
        }
        removeParsedData( buffer_, numBytesParsed );

        const bool gotMessage = (rxMsg!=0);
        if ( gotMessage )
        {
            try {
                rxMsgCallback_->msgReceived( rxMsg, timeStampSec, timeStampUsec );
                statusOK = true;
            }
            catch ( std::exception &e )
            {
                statusOK = false;
                stringstream ss;
                ss << "On receipt of rxMsg: " << rxMsg->toString() << ": " << e.what();
                subStatus_.warning( ss.str() );
            }
            catch ( ... )
            {
                statusOK = false;
                stringstream ss;
                ss << "On receipt of rxMsg: " << rxMsg->toString() << ": unknown exception";
                subStatus_.warning( ss.str() );
            }

            if ( rxMsg->isError() || rxMsg->isWarn() )
            {
                stringstream ss;
                ss << "SerialDeviceHandler: Received abnormal rxMsg: " << rxMsg->toString();
                subStatus_.warning( ss.str() );
                statusOK = false;
            }
            else
            {
                statusOK = true;
            }
        }
        else
        {
            break;
        }

        // If we've parsed the entire thing we can stop.
        if ( buffer_.size() == 0 )
        {
            break;
        }
    }
    return statusOK;
}

} // namespace
