/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2008-2010 Geoffrey Biggs
 *
 * hokuyo_aist Hokuyo laser scanner driver.
 *
 * This distribution is licensed to you under the terms described in the
 * LICENSE file included in this distribution.
 *
 * This work is a product of the National Institute of Advanced Industrial
 * Science and Technology, Japan. Registration number: H22PRO-1086.
 *
 * This file is part of hokuyo_aist.
 *
 * This software is licensed under the Eclipse Public License -v 1.0 (EPL). See
 * http://www.opensource.org/licenses/eclipse-1.0.txt
 */

#include "hokuyo_errors.h"

#include <cstdlib>
#include <cstring>

namespace hokuyo_aist
{

// error must be null-terminated
std::string scip2_error_to_string(char const* const error,
        char const* const cmd)
{
    std::stringstream ss;

    // Check for non-errors
    if(error[0] == '0' && error[1] == '0')
        return "Status OK - 00";
    else if(error[0] == '9' && error[1] == '9')
        return "Status OK - 99";

    // Check for universal errors
    if(error[0] == '0' && error[1] == 'A')
        return "Unable to create transmission data or reply command internally";
    else if(error[0] == '0' && error[1] == 'B')
        return "Buffer shortage or command repeated that is already processed";
    else if(error[0] == '0' && error[1] == 'C')
        return "Command with insufficient parameters 1";
    else if(error[0] == '0' && error[1] == 'D')
        return "Undefined command 1";
    else if(error[0] == '0' && error[1] == 'E')
        return "Undefined command 2";
    else if(error[0] == '0' && error[1] == 'F')
        return "Command with insufficient parameters 2";
    else if(error[0] == '0' && error[1] == 'G')
        return "String character in command exceeds 16 letters";
    else if(error[0] == '0' && error[1] == 'H')
        return "String character has invalid letters";
    else if(error[0] == '0' && error[1] == 'I')
        return "Sensor is now in firmware update mode";

    int error_code = atoi(error);

    if(cmd[0] == 'B' && cmd[1] == 'M')
    {
        switch(error_code)
        {
            case 1:
                return "Unable to control due to laser malfunction";
            case 2:
                return "Laser is already on";
        }
    }
// No info in the manual for this.
//    else if(cmd[0] == 'Q' && cmd[1] == 'T')
//    {
//        switch(error_code)
//        {
//            default:
//                std::stringstream ss;
//                ss << "Unknown error code " << error_code <<
//                      " for command " << cmd[0] << cmd[1];
//                return ss.str ();
//        }
//    }
    else if(((cmd[0] == 'G' || cmd[0] == 'H') &&
                (cmd[1] == 'D' || cmd[1] == 'E')) ||
             (cmd[0] == 'G' && cmd[1] == 'S'))
    {
        switch(error_code)
        {
            case 1:
                return "Starting step has non-numeric value";
            case 2:
                return "Ending step has non-numeric value";
            case 3:
                return "Cluster count has non-numeric value";
            case 4:
                return "Ending step is out of range";
            case 5:
                return "Ending step is smaller than start step";
            case 10:
                return "Laser is off";
            default:
                if(error_code >= 50)
                    ss << "Hardware error: " << error_code;
                else
                    ss << "Unknown error code " << error_code <<
                        " for command " << cmd[0] << cmd[1];

                return ss.str();
        }
    }
    else if(((cmd[0] == 'M' || cmd[0] == 'N') && 
                (cmd[1] == 'D' || cmd[1] == 'E')) ||
             (cmd[0] == 'M' && cmd[1] == 'S'))
    {
        switch(error_code)
        {
            case 1:
                return "Starting step has non-numeric value";
            case 2:
                return "Ending step has non-numeric value";
            case 3:
                return "Cluster count has non-numeric value";
            case 4:
                return "Ending step is out of range";
            case 5:
                return "Ending step is smaller than start step";
            case 6:
                return "Scan interval has non-numeric value";
            case 7:
                return "Number of scans is non-numeric";
            default:
                if(error_code >= 21 && error_code <= 49)
                {
                    return "Processing stopped to verify error. "
                        "This function is not yet supported by hokuyo_aist.";
                }
                else if(error_code >= 50 && error_code <= 97)
                    ss << "Hardware error: " << error_code;
                else if(error_code == 98)
                {
                    return "Resumption of processing after confirming normal "
                        "laser opteration. This function is not yet supported "
                        "by hokuyo_aist.";
                }
                else
                    ss << "Unknown error code " << error_code <<
                        " for command " << cmd[0] << cmd[1];

                return ss.str();
        }
    }
    else if(cmd[0] == 'T' && cmd[1] == 'M')
    {
        switch(error_code)
        {
            case 1:
                return "Invalid control code";
            case 2:
                return "Adjust mode on command received when sensor's adjust "
                    "mode is already on";
            case 3:
                return "Adjust mode off command received when sensor's adjust "
                    "mode is already off";
            case 4:
                return "Adjust mode is off when requested time";
        }
    }
    else if(cmd[0] == 'S' && cmd[1] == 'S')
    {
        switch(error_code)
        {
            case 1:
                return "Baud rate has non-numeric value";
            case 2:
                return "Invalid baud rate";
            case 3:
                return "Sensor is already running at that baud rate";
            case 4:
                return "Not compatible with the sensor model";
        }
    }
    else if(cmd[0] == 'C' && cmd[1] == 'R')
    {
        switch(error_code)
        {
            case 1:
                return "Invalid speed";
            case 2:
                return "Speed is out of range";
            case 3:
                return "Motor is already running at that speed";
            case 4:
                return "Not compatible with the sensor model";
        }
    }
    else if(cmd[0] == 'H' && cmd[1] == 'S')
    {
        switch(error_code)
        {
            case 1:
                return "Parameter error";
            case 2:
                return "Already running in the set mode";
            case 3:
                return "Not compatible with the sensor model";
        }
    }
// No info in the manual for this.
//    else if(cmd[0] == 'R' && cmd[1] == 'S')
//    {
//        switch(error_code)
//        {
//            case :
//                return "";
//            default:
//                std::stringstream ss;
//                ss << "Unknown error code " << error_code <<
//                    " for command " << cmd[0] << cmd[1];
//                return ss.str ();
//        }
//    }
// No info in the manual for this.
//    else if(cmd[0] == 'V' && cmd[1] == 'V')
//    {
//        switch(error_code)
//        {
//            case :
//                return "";
//        }
//    }
// No info in the manual for this.
//    else if(cmd[0] == 'P' && cmd[1] == 'P')
//    {
//        switch(error_code)
//        {
//            case :
//                return "";
//        }
//    }
// No info in the manual for this.
//    else if(cmd[0] == 'I' && cmd[1] == 'I')
//    {
//        switch(error_code)
//        {
//            case :
//                return "";
//        }
//    }
    else if(cmd[0] == 'D' && cmd[1] == 'B')
    {
        switch(error_code)
        {
            case 1:
                return "Parameter error";
            case 2:
                return "Already running in the set mode";
            case 3:
                return "Already returned to normal mode";
            case 4:
                return "Selected mode does not match the SCIP version in use";
            case 5:
                return "Sensor has a physical malfunction";
        }
    }
    else
    {
        ss << "Unknown command: " << cmd[0] << cmd[1];
        return ss.str();
    }

    // Known commands with unknown error codes fall through to here
    ss << "Unknown error code " << error_code << " for command " << cmd[0] <<
        cmd[1];
    return ss.str();
}


std::string desc_code_to_string(unsigned int code)
{
    static char const* const descriptions[] = {
/* 0 */  "Timed out trying to read a line",
/* 1 */  "No data received when trying to read a line.",
/* 2 */  "Invalid data index.",
/* 3 */  "Port is not open.",
/* 4 */  "Unknown SCIP version.",
/* 5 */  "Cannot change baud rate of non-serial connection.",
/* 6 */  "Bad baud rate: ",
/* 7 */  "SCIP version 1 does not support the reset command.",
/* 8 */  "SCIP version 1 does not support the set motor speed command.",
/* 9 */  "Invalid motor speed.",
/* 10 */ "SCIP version 1 does not support the high sensitivity command.",
/* 11 */ "No info object provided.",
/* 12 */ "SCIP version 1 does not support the get time command.",
/* 13 */ "No data received. Check data error code.",
/* 14 */ "Start step is out of range.",
/* 15 */ "End step is out of range.",
/* 16 */ "SCIP version 1 does not support the get new ranges command.",
/* 17 */ "SCIP version 1 does not support the get new ranges and intensities command.",
/* 18 */ "Timed out while skipping.",
/* 19 */ "Failed to write command byte.",
/* 20 */ "Failed to write command parameters.",
/* 21 */ "Failed to write termination character.",
/* 22 */ "SCIP versions 1 and 2 failed.",
/* 23 */ "Out-of-range firmware version.",
/* 24 */ "Invalid checksum: ",
/* 25 */ "Read a different number of range or intensity readings than were asked for.",
/* 26 */ "Found line feed in a data block.",
/* 27 */ "Unknown line: ",
/* 28 */ "Parse error: ",
/* 29 */ "'FIRM:' was not found when checking firmware version.",
/* 30 */ "Bad response.",
/* 31 */ "Incorrect command echo.",
/* 32 */ "Incorrect parameters echo for command.",
/* 33 */ "Not enough bytes to calculate checksum.",
/* 34 */ "Incorrect line length received.",
/* 35 */ "SCIP version 1 does not support the semi-reset command.",
/* 36 */ "SCIP version 1 does not support the get ranges and intensities command.",
/* 37 */ "Error configuring IP address.",
/* 38 */ "Did not receive a full line."
    };

    return std::string(descriptions[code]);
}


BaseError::BaseError(unsigned int desc_code, char const* error_type)
    : desc_code_(desc_code)
{
    strncpy(error_type_, error_type, 32);
}


BaseError::BaseError(BaseError const& rhs)
    : desc_code_(rhs.desc_code())
{
    strncpy(error_type_, rhs.error_type(), 32);
}


const char* BaseError::what() throw()
{
    ss << error_type_ << " (" << desc_code_ << "): " <<
        desc_code_to_string(desc_code_);
    return ss.str().c_str();
}


const char* BaudrateError::what() throw()
{
    RuntimeError::what();
    ss << baud_;
    return ss.str().c_str();
}


const char* ChecksumError::what() throw()
{
    ProtocolError::what();
    ss << "expected " << expected_ << ", calculated " << calculated_;
    return ss.str().c_str();
}


UnknownLineError::UnknownLineError(char const* const line)
    : ProtocolError(27, "UnknownLineError")
{
    strncpy(line_, line, 128);
}


UnknownLineError::UnknownLineError(UnknownLineError const& rhs)
    : ProtocolError(rhs)
{
    strncpy(line_, rhs.line(), 128);
}


const char* UnknownLineError::what() throw()
{
    ProtocolError::what();
    ss << line_;
    return ss.str().c_str();
}


ParseError::ParseError(char const* const line, char const* const type)
    : ProtocolError(28, "ParseError")
{
    strncpy(line_, line, 128);
    strncpy(type_, type, 16);
}


ParseError::ParseError(ParseError const& rhs)
    : ProtocolError(rhs)
{
    strncpy(line_, rhs.line(), 128);
    strncpy(type_, rhs.type(), 16);
}


const char* ParseError::what() throw()
{
    ProtocolError::what();
    ss << "Line type: " << type_ << ". Line: " << line_;
    return ss.str().c_str();
}


const char* ResponseError::what() throw()
{
    ProtocolError::what();
    ss << " Command: " << cmd_[0] << cmd_[1];
    ss << " Error : (" << error_[0] << error_[1] << ") " <<
        scip2_error_to_string(error_, cmd_);
    return ss.str().c_str();
}


const char* Scip1ResponseError::what() throw()
{
    ProtocolError::what();
    ss << " Command: " << cmd_;
    ss << " Error : " << error_;
    return ss.str().c_str();
}


const char* CommandEchoError::what() throw()
{
    ProtocolError::what();
    ss << " Command: " << cmd_[0] << cmd_[1];
    ss << " Received echo: " << echo_[0] << echo_[1];
    return ss.str().c_str();
}


const char* ParamEchoError::what() throw()
{
    ProtocolError::what();
    ss << " Command: " << cmd_[0] << cmd_[1];
    return ss.str().c_str();
}


const char* InsufficientBytesError::what() throw()
{
    ProtocolError::what();
    ss << " Number of bytes: " << num_;
    ss << " Line length: " << line_length_;
    return ss.str().c_str();
}


const char* LineLengthError::what() throw()
{
    ProtocolError::what();
    ss << " Received length: " << length_;
    ss << " Expected line length: " << expected_;
    return ss.str().c_str();
}

}; // namespace hokuyo_aist

