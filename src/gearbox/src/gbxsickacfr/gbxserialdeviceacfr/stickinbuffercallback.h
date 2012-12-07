#ifndef GBXSERIALDEVICEACFR_STICKINBUFFERCALLBACK_H
#define GBXSERIALDEVICEACFR_STICKINBUFFERCALLBACK_H

#include <gbxsickacfr/gbxserialdeviceacfr/serialdevicehandler.h>
#include <gbxsickacfr/gbxiceutilacfr/buffer.h>

namespace gbxserialdeviceacfr {

//! Received message plus a timeStamp: a simple container to keep the two together
class TimedRxMsg {
public:

    // Require an empty constructor to put in a buffer
    TimedRxMsg() {}
    TimedRxMsg( int s, int us, const RxMsgPtr &r )
        : timeStampSec(s), timeStampUsec(us), msg(r) {}

    int timeStampSec;
    int timeStampUsec;
    RxMsgPtr msg;
};

//
// @brief simply sticks new messages into a thread-safe buffer
//
// @author Alex Brooks
//
class StickInBufferCallback : public RxMsgCallback
{
public: 

    StickInBufferCallback() {}

    // from RxMsgCallback
    void msgReceived( const RxMsgPtr &msg,
                      int             timeStampSec,
                      int             timeStampUsec )
        { rxMsgBuffer_.push( TimedRxMsg( timeStampSec, timeStampUsec, msg ) ); }

    // Allow external non-const access direct to (thread-safe) rxMsgBuffer.
    gbxiceutilacfr::Buffer<TimedRxMsg> &rxMsgBuffer() { return rxMsgBuffer_; }

private: 

    // Thread-safe store of rxMsgs from the device
    gbxiceutilacfr::Buffer<TimedRxMsg> rxMsgBuffer_;
};

}

#endif
