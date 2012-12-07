/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBXSERIALDEVICEACFR_SERIALDEVICEHANDLER_H
#define GBXSERIALDEVICEACFR_SERIALDEVICEHANDLER_H

#include <gbxserialacfr/serial.h>
#include <gbxutilacfr/substatus.h>
#include <gbxsickacfr/gbxiceutilacfr/safethread.h>
#include <gbxsickacfr/gbxiceutilacfr/store.h>
#include <IceUtil/IceUtil.h>

namespace gbxserialdeviceacfr {

//! @brief A base-class for a message received from the device on the other end
class RxMsg : public IceUtil::Shared
{
public:
    ~RxMsg() {}
    
    // Does the message indicate a warning condition?
    // If so, this will be reported to SerialDeviceHandler's status.
    virtual bool isWarn() const { return false; }
    // Does the message indicate an error condition?
    // If so, this will be reported to SerialDeviceHandler's status.
    virtual bool isError() const { return false; }
    
    // Human-readable string
    virtual std::string toString() const=0;
};
typedef IceUtil::Handle<RxMsg> RxMsgPtr;

//!
//! The implementation of this class needs to be provided by the user.
//! It parses buffers to produce discrete messages (messages received from the device)
//!
class RxMsgParser {
public:
    virtual ~RxMsgParser() {}

    // Parses the contents of the buffer.
    // Params:
    //   - 'numBytesParsed': this function sets this to the number of bytes parsed
    //                       (irrespective of whether or not a valid RxMsg was found)
    // Returns:
    //   - the parsed received message, if it is found, or
    //   - NULL if no message is found.
    virtual RxMsgPtr parseBuffer( const std::vector<char> &buffer,
                                  int                     &numBytesParsed ) = 0;
};

//!
//! The implementation of this class needs to be provided by the user.
//! It is called when a new message is received.
//!
//! Be aware that this callback is called in the message-reading
//! thread -- if you hog it, no more messages will arrive till you
//! un-hog it.
//!
class RxMsgCallback {
public:
    virtual ~RxMsgCallback() {}

    virtual void msgReceived( const RxMsgPtr &msg,
                              int             timeStampSec,
                              int             timeStampUsec )=0;
};

//!
//! @brief Handles the serial port.
//!
//! This thread waits for data to arrive from the device, and calls
//! RxMsgParser::parseBuffer on that data.
//!
//! Do this in this separate thread so we can hopefully grab the messages
//! as soon as they arrive, without relying on an external poller
//! which may be busy doing other stuff.
//! This will hopefully give us accurate timestamps.
//!
//! @author Alex Brooks
//!
class SerialDeviceHandler : public gbxiceutilacfr::SafeThread
{

public: 

    // Params:
    //   - subsysName: given to Status
    //   - unparsedBytesWarnThreshold: if we get more than this many un-parsed bytes packed into the
    //                                 receive buffer, flag a warning.
    //   - serialPort_: must have timeouts enabled
    SerialDeviceHandler( const std::string     &subsysName,
                         gbxserialacfr::Serial &serialPort,
                         RxMsgParser           &rxMsgParser,
                         gbxutilacfr::Tracer   &tracer,
                         gbxutilacfr::Status   &status,
                         bool                   verboseDebug=false,
                         int                    unparsedBytesWarnThreshold = 20000 );
    void setRxMsgCallback( RxMsgCallback &callback )
        { rxMsgCallback_ = &callback; }

    ~SerialDeviceHandler();

    // Send the bytes to the device
    void send( const char* commandBytes, int numCommandBytes );

    // allows changing of baud rates on-the-fly
    void setBaudRate( int baudRate );

    // The SerialDeviceHandler runs in its own thread -- this lets you ensure that it's really running.
    void startAndWaitTillIsRunning();

private: 

    // The main thread function, inherited from SubsystemThread
    virtual void walk();

    // interrogate whether or not that thread is running.
    bool isRunning();

    // Returns: true if got data, false if timed out
    bool getDataFromSerial();
    // Returns: true if statusOK, false it something bad happened
    bool processBuffer();

    gbxserialacfr::Serial &serial_;

    // Knows how to parse for rxMsgs
    RxMsgParser &rxMsgParser_;
    // Knows what to do with them
    RxMsgCallback *rxMsgCallback_;

    // Contains un-parsed data from the device
    std::vector<char> buffer_;
    
    gbxiceutilacfr::Store<bool> isRunningStore_;

    int unparsedBytesWarnThreshold_;

    gbxutilacfr::Tracer& tracer_;
    gbxutilacfr::SubStatus subStatus_;

    const bool verboseDebug_;
};
//! A smart pointer to the class.
typedef IceUtil::Handle<SerialDeviceHandler> SerialDeviceHandlerPtr;

} // namespace

#endif
