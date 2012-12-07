/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBX_NOVATEL_RECEIVER_STATUS_DECODER_H
#define GBX_NOVATEL_RECEIVER_STATUS_DECODER_H

#include <stdint.h>
#include <string>
#include <sstream>
namespace gbxnovatelutilacfr{
bool receiverStatusIsGood(uint32_t receiverStatus){
    return 0 == (receiverStatus & 0xe1fe8fef);          // quick cross-check: this magic value needs to be the sum of the magic values in the next three functions
}
bool receiverStatusIsWarning(uint32_t receiverStatus){
    return 0 != (receiverStatus & 0xe1fc000e);
}
bool receiverStatusIsError(uint32_t receiverStatus){
    return 0 != (receiverStatus & 0x00028fe0);
}
bool receiverStatusIsFatal(uint32_t receiverStatus){
    return 0 != (receiverStatus & 0x00000001);
}
bool receiverStatusIsReservedValue(uint32_t receiverStatus){
    return 0 != (receiverStatus & 0x1e017010);          // quick cross-check: this magic value, summed with the one from receiverStatusIsGood() should yield 0xffffffff
}
std::string receiverStatusToString(uint32_t receiverStatus){
    std::stringstream ss;
    ss << "Error flag: "
        << ((0 == (receiverStatus & 0x00000001)) ? "No error" : "Error") << "; ";               // unrecoverable --> exception
    ss << "Temperature status: "
        << ((0 == (receiverStatus & 0x00000002)) ? "Within specifications" : "Warning") << "; ";// warning (error?)
    ss << "Voltage supply status: "
        << ((0 == (receiverStatus & 0x00000004)) ? "OK" : "Warning") << "; ";                   // warning (error?)
    ss << "Antenna power status: "
        << ((0 == (receiverStatus & 0x00000008)) ? "Powered" : "Not powered") << "; ";          // warning (are there unpowered antennas??)
    ss << "Antenna open flag: "
        << ((0 == (receiverStatus & 0x00000020)) ? "OK" : "Open") << "; ";                      // error
    ss << "Antenna shorted flag: "
        << ((0 == (receiverStatus & 0x00000040)) ? "OK" : "Shorted") << "; ";                   // error
    ss << "CPU overload flag: "
        << ((0 == (receiverStatus & 0x00000080)) ? "No overload" : "Overload") << "; ";         // error (recoverable? warning?)
    ss << "COM1 buffer overrun flag: "
        << ((0 == (receiverStatus & 0x00000100)) ? "No overrun" : "Overrun") << "; ";           // error (recoverable? warning?)
    ss << "COM2 buffer overrun flag: "
        << ((0 == (receiverStatus & 0x00000200)) ? "No overrun" : "Overrun") << "; ";           // error (recoverable? warning?)
    ss << "COM3 buffer overrun flag: "
        << ((0 == (receiverStatus & 0x00000400)) ? "No overrun" : "Overrun") << "; ";           // error (recoverable? warning?)
    ss << "USB buffer overrun flag: "
        << ((0 == (receiverStatus & 0x00000800)) ? "No overrun" : "Overrun") << "; ";           // error (recoverable? warning?)
    ss << "RF1 AGC status: "
        << ((0 == (receiverStatus & 0x00008000)) ? "OK" : "Bad") << "; ";                       // error
    ss << "RF2 AGC status: "
        << ((0 == (receiverStatus & 0x00020000)) ? "OK" : "Bad") << "; ";                       // error
    ss << "Almanac flag/UTC known: "
        << ((0 == (receiverStatus & 0x00040000)) ? "Valid" : "Invalid") << "; ";                //warning
    ss << "Position solution flag: "
        << ((0 == (receiverStatus & 0x00080000)) ? "Valid" : "Invalid") << "; ";                //warning
    ss << "Position fixed flag: "
        << ((0 == (receiverStatus & 0x00100000)) ? "Not" : "fixed Fixed") << "; ";              //warning
    ss << "Clock steering status: "
        << ((0 == (receiverStatus & 0x00200000)) ? "Enabled" : "Disabled") << "; ";             //warning
    ss << "Clock model flag: "
        << ((0 == (receiverStatus & 0x00400000)) ? "Valid" : "Invalid") << "; ";                //warning
    ss << "OEMV card external oscillator flag: "
        << ((0 == (receiverStatus & 0x00800000)) ? "Disabled" : "Enabled") << "; ";             //warning (is this really a warning??)
    ss << "Software resource: "
        << ((0 == (receiverStatus & 0x01000000)) ? "OK" : "Warning") << "; ";                   //warning (error?)
    ss << "Auxiliary 3 status event flag: "
        << ((0 == (receiverStatus & 0x20000000)) ? "No event" : "Event") << "; ";               //warning
    ss << "Auxiliary 2 status event flag: "
        << ((0 == (receiverStatus & 0x40000000)) ? "No event" : "Event") << "; ";               //warning
    ss << "Auxiliary 1 status event flag: "
        << ((0 == (receiverStatus & 0x80000000)) ? "No event" : "Event");                       //warning
    return ss.str();
}
}//namespace

#endif
