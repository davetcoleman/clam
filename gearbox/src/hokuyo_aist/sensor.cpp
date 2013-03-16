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

#include "hokuyo_aist.h"
#include "hokuyo_errors.h"
#include "sensor_info.h"
#include "utils.h"
using namespace hokuyo_aist;

#include <flexiport/flexiport.h>
#include <flexiport/port.h>
#include <flexiport/serialport.h>

#include <cassert>
#include <cstring>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <fstream>

#if defined(WIN32)
    #define __func__    __FUNCTION__
#endif

namespace hokuyo_aist
{

// SCIP1: 66 bytes (64 bytes of data + line feed + 0)
unsigned int const SCIP1_LINE_LENGTH = 66;
// SCIP2: 67 bytes (64 bytes of data + checksum byte + line feed + 0)
unsigned int const SCIP2_LINE_LENGTH = 67;
// Size of a full data block in range data
unsigned int const DATA_BLOCK_LENGTH = 64;

///////////////////////////////////////////////////////////////////////////////
// SCIP protocol version 1 notes
///////////////////////////////////////////////////////////////////////////////

/* | = byte boundary, ... indicates variable byte block (max 64 bytes),
(x) = x byte block
 - No checksum
 - Host to sensor: Command | Parameters... | LF
 - Sensor to host: Command | Parameters... | LF | Status | LF | Data... | LF | LF
 - Where a block of data would take more than 64 bytes, a line feed is inserted
   every 64 bytes.
 - Status 0 is OK, anything else is an error.

L  Power
   L|Control code|LF
   L|Control code|LF|Status|LF|LF
   3 byte command block
G  Get data
   G|Start(3)|End(3)|Cluster(2)|LF
   G|Start(3)|End(3)|Cluster(2)|LF|Status|LF|Data...|LF|LF
   10 byte command block
S  Set baud rate
   S|Baud rate(6)|Reserved(7)|LF
   S|Baud rate(6)|Reserved(7)|Status|LF|LF|
   16 byte command block
V  Version info
   V|LF
   V|LF|Status...|LF|Vendor...|LF|Product...|LF|Firmware...|LF|Protocol...|LF|
        Serial...|LF|LF
   2 byte command block
*/

// SCIP protocol version 2 notes
///////////////////////////////////////////////////////////////////////////////

/* | = byte boundary, ... indicates variable byte block (max 64 bytes),
(x) = x byte block
 - We don't use the string block (which can be up to 16 bytes) so it's marked
   as size 0 and ignored in the command definitions below.
 - Host to sensor: Command(2) | Parameters... | String(0) | LF
 - Sensor to host: Command(2) | Parameters... | String(0) | LF | Status(2) |
        Sum | LF
 - Each data row: Data (max 64) | Sum | LF
 - Data rows are broken after a maximum of 64 bytes, each one having a checksum
   and a line feed.
 - Status codes 00 and 99 are OK, anything else is an error.
 - Checksum is calculated by... well, see the code.
 - BIG NOTE: The UTM-30LX has a probable bug in its response to the II code
   whereby it does not include anything after and including "<-" in the
   checksum calculation.

VV    Version info
      V|V|LF
      V|V|LF|Status(2)|Sum|LF|Vendor...|;|Sum|LF|Product...|;|Sum|LF|
        Firmware...|;|Sum|LF|Protocol...|;|Sum|LF|Serial...|;|Sum|LF|LF
      3 byte command block
PP    Specification info
      P|P|LF
      P|P|LF|Status(2)|Sum|LF|Model...|;|Sum|LF|MinRange...|;|Sum|LF|
        MaxRange...|;|Sum|LF|TotalSteps...|;|Sum|LF|FirstStep...|;|Sum|LF|
        LastStep...|;|Sum|LF|FrontStep...|;|Sum|LF|MotorSpeed...|;|Sum|LF|LF
      3 byte command block
II    Status info
      I|I|LF
      I|I|LF|Status(2)|Sum|LF|Model...|;|Sum|LF|Power...|;|Sum|LF|
        MotorSpeed...|;|Sum|LF|Mode...|;|Sum|LF|Baud...|;|Sum|LF|Time...|;|Sum|
        LF|Diagnostic...|;|Sum|LF|LF
      3 byte command block
BM    Power on
      B|M|LF
      B|M|LF|Status(2)|Sum|LF|LF
      3 byte command block
QT    Power off
      Q|T|LF
      Q|T|LF|Status(2)|Sum|LF|LF
      3 byte command block
SS    Set baud rate
      S|S|Baud(6)|LF
      S|S|Baud(6)|LF|Status(2)|Sum|LF|LF
      9 byte command block
MDMS  Get new data
      M|D/S|Start(4)|End(4)|Cluster(2)|Interval(1)|Number(2)|LF
      M|D/S|Start(4)|End(4)|Cluster(2)|Interval(1)|Number remaining(2)|LF|
        Status(2)|Sum|LF|Data...|LF|LF
      16 byte command block
ME    Get new data, including intensity data
      M|E|Start(4)|End(4)|Cluster(2)|Interval(1)|Number(2)|LF
      M|E|Start(4)|End(4)|Cluster(2)|Interval(1)|Number remaining(2)|LF|
        Status(2)|Sum|LF|Data...|LF|LF
      16 byte command block
GDGS  Get latest data
      G|D/S|Start(4)|End(4)|Cluster(2)|LF
      G|D/S|Start(4)|End(4)|Cluster(2)|LF|Status(2)|Sum|LF|Data...|LF|LF
      12 byte command block
GE    Get latest data, including intensity data
      G|E|Start(4)|End(4)|Cluster(2)|LF
      G|E|Start(4)|End(4)|Cluster(2)|LF|Status(2)|Sum|LF|Data...|LF|LF
      12 byte command block
CR    Set motor speed
      C|R|Speed(2)|LF
      C|R|Speed(2)|LF|Status(2)|Sum|LF|LF
      5 byte command block
TM    Get sensor time
      T|M|Code|LF
      T|M|Code|LF|Status(2)|Sum|LF[|Time(4)|Sum|LF|LF
      4 byte command block
      Optional part only comes back for control code 1.
RS    Reset
      R|S|LF
      R|S|LF|Status(2)|Sum|LF|LF
      3 byte command block
RT    Same as reset, but does not stop the motor or alter serial speed
      R|T|LF
      R|T|LF|Status(2)|Sum|LF|LF
ND    Multi-echo version of MD
      N|D|Start(4)|End(4)|Cluster(2)|Interval(1)|Number(2)|LF
      N|D|Start(4)|End(4)|Cluster(2)|Interval(1)|Number remaining(2)|LF|
        Status(2)|Sum|LF|Data...|LF|LF
      16 byte command block
      Returned scan data may include echo data separated by '&'.
NE    Multi-echo version of ME
      N|E|Start(4)|End(4)|Cluster(2)|Interval(1)|Number(2)|LF
      N|E|Start(4)|End(4)|Cluster(2)|Interval(1)|Number remaining(2)|LF|
        Status(2)|Sum|LF|Data...|LF|LF
      16 byte command block
      Returned scan data may include echo data separated by '&'.
HD    Multi-echo version of GD
      H|D|Start(4)|End(4)|Cluster(2)|LF
      H|D|Start(4)|End(4)|Cluster(2)|LF|Status(2)|Sum|LF|Data...|LF|LF
      12 byte command block
      Returned scan data may include echo data separated by '&'.
HE    Multi-echo version of GE
      H|E|Start(4)|End(4)|Cluster(2)|LF
      H|E|Start(4)|End(4)|Cluster(2)|LF|Status(2)|Sum|LF|Data...|LF|LF
      12 byte command block
      Returned scan data may include echo data separated by '&'.
HS    Set high sensitivity mode
      H|S|On/Off(1)|LF
      H|S|On/Off(1)|LF|Status(2)|Sum|LF|LF
      3 byte command block
      Use '0' for on, '1' for off.
$IP   Change IP address settings
      $|I|P|IP(15)| |Subnet mask(15)| |Default gateway(15)|LF
      $|I|P|IP(15)| |Subnet mask(15)| |Default gateway(15)|LF|Status(2)|
        Sum|LF|LF
      Only available on models supporting ethernet connections.
      Does not conform to standard command format.
      Laser will require a restart after use (wait for response first).
DB    Simulate sensor failure modes
      D|B|Command(2)|LF
      D|B|Command(2)|LF|Status(2)|Sum|LF|LF
      Where command is one of:
      "02" Enter failure state.
      "03" While sending data, go through normal -> malfunction -> normal.
      "04" While sending data, go through normal -> malfunction -> failure.
      "05" While sending data, go through normal -> failure.
      "10" Return to normal operation.
      4 byte command block
*/

///////////////////////////////////////////////////////////////////////////////
// Utility functions
///////////////////////////////////////////////////////////////////////////////

unsigned int decode_2_byte_value(char* data)
{
    unsigned int byte1, byte2;

    byte1 = data[0] - 0x30;
    byte2 = data[1] - 0x30;

    return (byte1 << 6) + (byte2);
}


unsigned int decode_3_byte_value(char* data)
{
    unsigned int byte1, byte2, byte3;

    byte1 = data[0] - 0x30;
    byte2 = data[1] - 0x30;
    byte3 = data[2] - 0x30;

    return (byte1 << 12) + (byte2 << 6) + (byte3);
}


unsigned int decode_4_byte_value(char* data)
{
    unsigned int byte1, byte2, byte3, byte4;

    byte1 = data[0] - 0x30;
    byte2 = data[1] - 0x30;
    byte3 = data[2] - 0x30;
    byte4 = data[3] - 0x30;

    return (byte1 << 18) + (byte2 << 12) + (byte3 << 6) + (byte4);
}


void number_to_string(unsigned int num, char* dest, int length)
{
#if defined(WIN32)
    _snprintf(dest, length + 1, "%*d", length, num);
#else
    snprintf(dest, length + 1, "%*d", length, num);
#endif
    // Replace all leading spaces with '0'
    for (int ii = 0; ii < length && dest[ii] == ' '; ii++)
        dest[ii] = '0';
}


///////////////////////////////////////////////////////////////////////////////
// Sensor class
///////////////////////////////////////////////////////////////////////////////

// Public API
///////////////////////////////////////////////////////////////////////////////

Sensor::Sensor()
    : port_(0), err_output_(std::cerr), scip_version_(2), verbose_(false),
    enable_checksum_workaround_(false), ignore_unknowns_(false),
    multiecho_mode_(ME_OFF), min_angle_(0.0), max_angle_(0.0),
    resolution_(0.0), first_step_(0), last_step_(0), front_step_(0),
    max_range_(0), time_resolution_(0), time_offset_(0), last_timestamp_(0),
    wrap_count_(0), time_drift_rate_(0.0), time_skew_alpha_(0.0)
{
}


Sensor::Sensor(std::ostream& err_output)
    : port_(0), err_output_(err_output), scip_version_(2), verbose_(false),
    enable_checksum_workaround_(false), ignore_unknowns_(false),
    multiecho_mode_(ME_OFF), min_angle_(0.0), max_angle_(0.0),
    resolution_(0.0), first_step_(0), last_step_(0), front_step_(0),
    max_range_(0), time_resolution_(0), time_offset_(0), last_timestamp_(0),
    wrap_count_(0), time_drift_rate_(0.0), time_skew_alpha_(0.0)
{
}


Sensor::~Sensor()
{
    if(port_ != 0)
     delete port_;
}


void Sensor::open(std::string port_options)
{
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() Creating and opening port using options: " <<
            port_options << '\n';
    }
    port_ = flexiport::CreatePort(port_options);
    port_->Open();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Connected using " <<
            port_->GetPortType() << " connection.\n";
        err_output_ << port_->GetStatus();
    }
    port_->Flush();

    // Figure out the SCIP version currently in use and switch to a higher one
    // if possible
    get_and_set_scip_version();
    // Get some values we need for providing default ranges
    get_defaults();
}


unsigned int Sensor::open_with_probing(std::string port_options)
{
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() Creating and opening port using options: " << port_options <<
            '\n';
    }
    port_ = flexiport::CreatePort(port_options);
    port_->Open();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Connected using " <<
            port_->GetPortType() << " connection.\n";
        err_output_ << port_->GetStatus();
    }
    port_->Flush();

    try
    {
        // Figure out the SCIP version currently in use and switch to a higher
        // one if possible
        get_and_set_scip_version();
        // Get some values we need for providing default ranges
        get_defaults();
    }
    catch(BaseError)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Failed to connect at the default baud rate.\n";
        }
        if(port_->GetPortType() == "serial")
        {
            // Failed at the default baud rate, so try again at the other
            // rates. Note that a baud rate of 750000 or 250000 doesn't appear
            // to be supported on any common OS.
            unsigned int const bauds[] = {500000, 115200, 57600, 38400, 19200};
            unsigned int const numBauds(5);
            for (unsigned int ii = 0; ii < numBauds; ii++)
            {
                reinterpret_cast<flexiport::SerialPort*>(port_)->SetBaudRate(bauds[ii]);
                try
                {
                    get_and_set_scip_version();
                    get_defaults();
                    // If the above two functions succeed, break out of the
                    // loop and be happy
                    if(verbose_)
                    {
                        err_output_ << "Sensor::" << __func__ <<
                            "() Connected at " << bauds[ii] << '\n';
                    }
                    return bauds[ii];
                }
                catch(BaseError)
                {
                    if(ii == numBauds - 1)
                    {
                        // Last baud rate, give up and rethrow
                        if(verbose_)
                        {
                            err_output_ << "Sensor::" << __func__ <<
                                "() Failed to connect at any baud rate.\n";
                        }
                        throw;
                    }
                    // Otherwise go around again
                }
            }
        }
        else
        {
            if(verbose_)
            {
                err_output_ << "Sensor::" << __func__ <<
                    "() Port is not serial, cannot probe.\n";
            }
            throw;
        }
    }

    if(port_->GetPortType() == "serial")
        return reinterpret_cast<flexiport::SerialPort*>(port_)->GetBaudRate();
    else
        return 0;
}


void Sensor::close()
{
    if(!port_)
        throw CloseError();
    if(verbose_)
        err_output_ << "Sensor::" << __func__ << "() Closing connection.\n";
    delete port_;
    port_ = 0;
}


bool Sensor::is_open() const
{
    if(port_ != 0)
        return port_->IsOpen();
    return false;
}


void Sensor::set_power(bool on)
{
    if(scip_version_ == 1)
    {
        if(on)
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Turning laser on.\n";
            send_command("L", "1", 1, 0);
        }
        else
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Turning laser off.\n";
            send_command("L", "0", 1, 0);
        }
        skip_lines(1);
    }
    else if(scip_version_ == 2)
    {
        if(on)
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Turning laser on.\n";
            send_command("BM", 0, 0, "02");
        }
        else
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Turning laser off.\n";
            send_command("QT", 0, 0, "02");
        }
        skip_lines(1);
    }
    else
        throw UnknownScipVersionError();
}


// This function assumes that both the port and the laser scanner are already
// set to the same baud.
void Sensor::set_baud(unsigned int baud)
{
    if(port_->GetPortType() != "serial")
        throw NotSerialError();

    char newBaud[13];
    memset(newBaud, 0, sizeof(char) * 13);

    if(baud != 19200 && baud != 38400 && baud != 57600 && baud != 115200 &&
        baud != 250000 && baud != 500000 && baud != 750000)
    {
        throw BaudrateError(baud);
    }
    number_to_string(baud, newBaud, 6);

    if(scip_version_ == 1)
    {
        // Send the command to change baud rate
        send_command("S", newBaud, 13, 0);
        skip_lines(1);
        // Change the port's baud rate
        reinterpret_cast<flexiport::SerialPort*>(port_)->SetBaudRate(baud);
    }
    else if(scip_version_ == 2)
    {
        // Send the command to change baud rate
        send_command("SS", newBaud, 6, "03");
        skip_lines(1);
        // Change the port's baud rate
        reinterpret_cast<flexiport::SerialPort*>(port_)->SetBaudRate(baud);
    }
    else
        throw UnknownScipVersionError();
}


void Sensor::set_ip(IPAddr const& addr, IPAddr const& subnet,
        IPAddr const& gateway)
{
    // Command is "$IP"(3) + IP(15) + ' ' + subnet(15) + ' ' + gateway(15),
    // +1 for the null byte, but split two bytes off the front for sending.
    // Hijack the send_command_ function, treating the rest of the command as
    // parameters.
    char const command[3] = "$I";
    std::stringstream params;
    params << 'P';
    params << std::setw(3) << std::setfill('0');
    params << addr.first << '.' << addr.second << '.' << addr.third << '.' <<
        addr.fourth << ' ';
    params << subnet.first << '.' << subnet.second << '.' << subnet.third <<
        '.' << subnet.fourth << ' ';
    params << gateway.first << '.' << gateway.second << '.' << gateway.third <<
        '.' << gateway.fourth;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() Setting IP information to $I" << params.str() << '\n';
    }
    int status = send_command(&command[0], params.str().c_str(), 48, 0);
    // Skip the extra line feed
    skip_lines(1);
    if(status != 0)
        throw SetIPError();
}


void Sensor::reset()
{
    if(scip_version_ == 1)
        throw UnsupportedError(7);
    else if(scip_version_ == 2)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Resetting laser.\n";
        }
        send_command("RS", 0, 0, 0);
        skip_lines(1);
    }
    else
        throw UnknownScipVersionError();
}


void Sensor::semi_reset()
{
    if(scip_version_ == 1)
        throw UnsupportedError(35);
    else if(scip_version_ == 2)
    {
        if(verbose_)
            err_output_ << "Sensor::" << __func__ <<
                "() Resetting laser.\n";
        send_command("RS", 0, 0, 0);
        skip_lines(1);
    }
    else
        throw UnknownScipVersionError();
}


void Sensor::set_motor_speed(unsigned int speed)
{
    if(scip_version_ == 1)
        throw UnsupportedError(8);
    else if(scip_version_ == 2)
    {
        // Sanity check the value
        if(speed > 10 && speed != 99)
            throw MotorSpeedError();
        char buffer[3];
        if(speed == 0)
        {
            if(verbose_)
            {
                err_output_ << "Sensor::" << __func__ <<
                    "() Reseting motor speed to default.\n";
            }
            buffer[0] = '0';
            buffer[1] = '0';
            buffer[2] = '\0';
        }
        else if(speed == 99)
        {
            if(verbose_)
            {
                err_output_ << "Sensor::" << __func__ <<
                    "() Reseting motor speed to default.\n";
            }
            buffer[0] = '9';
            buffer[1] = '9';
            buffer[2] = '\0';
        }
        else
        {
            if(verbose_)
            {
                err_output_ << "Sensor::" << __func__ <<
                    "() Setting motor speed to ratio " << speed << '\n';
            }
            number_to_string(speed, buffer, 2);
        }
        send_command("CR", buffer, 2, "03");
        skip_lines(1);
    }
    else
        throw UnknownScipVersionError();
}


void Sensor::set_high_sensitivity(bool on)
{
    if(scip_version_ == 1)
        throw UnsupportedError(10);
    else if(scip_version_ == 2)
    {
        if(on)
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Switching to high sensitivity.\n";
            send_command("HS", "1", 1, "02");
        }
        else
        {
            if(verbose_)
            {
                err_output_ << "Sensor::" << __func__ <<
                    "() Switching to normal sensitivity.\n";
            }
            send_command("HS", "0", 1, "02");
        }
        skip_lines(1);
    }
    else
        throw UnknownScipVersionError();
}


void Sensor::get_sensor_info(SensorInfo& info)
{
    if(scip_version_ == 1)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Getting sensor information using SCIP version 1.\n";
        }

        info.set_defaults();

        char buffer[SCIP1_LINE_LENGTH];
        memset(buffer, 0, sizeof(char) * SCIP1_LINE_LENGTH);

        send_command("V", 0, 0, 0);
        // Get the vendor info line
        read_line(buffer);
        info.vendor = &buffer[5]; // Chop off the "VEND:" tag
        // Get the product info line
        read_line(buffer);
        info.product = &buffer[5];
        // Only the URG-04LX supports SCIP1
        model_ = MODEL_URG04LX;
        // Get the firmware line
        read_line(buffer);
        info.firmware = &buffer[5];
        // Get the protocol version line
        read_line(buffer);
        info.protocol = &buffer[5];
        // Get the serial number
        read_line(buffer);
        info.serial = &buffer[5];
        // Get either the status line or the end of message
        read_line(buffer);
        if(buffer[0] != '\0')
        {
            // Got a status line
            info.sensor_diagnostic = &buffer[5];
            skip_lines(1);
        }

        // Check the firmware version major number. If it's >=3 there is
        // probably some extra info in the firmware line.
        // eg: FIRM:3.1.04,07/08/02(20-4095[mm],240[deg],44-725[step],600[rpm])
        // Note that this example is right up against the maximum SCIP v1 line
        // length of 64 bytes.
        if(atoi(info.firmware.c_str()) >= 3)
        {
            if(verbose_)
                err_output_ << "SCIP1 Firmware line for parsing: " <<
                    info.firmware << '\n';
            // Now the fun part: parsing the line. It would be nice if we could
            // use the POSIX regex functions, but since MS doesn't believe in
            // POSIX we get to do it the hard way.
            // Start by finding the first (
            char const* valueStart;
            if((valueStart = strchr(info.firmware.c_str(), '(')) == 0)
            {
                // No bracket? Crud. Fail and use the hard-coded values from
                // the manual.
                info.calculate_values();
            }
            // Now put it through sscanf and hope...
            int aperture;
            int numFound = sscanf(valueStart,
                    "(%d-%d[mm],%d[deg],%d-%d[step],%d[rpm]", &info.min_range,
                    &info.max_range, &aperture, &info.first_step,
                    &info.last_step, &info.speed);
            if(numFound != 6)
            {
                // Didn't get enough values out, assume unknown format and fall
                // back on the defaults
                info.set_defaults();
                info.calculate_values();
                if(verbose_)
                {
                    err_output_ << "Retrieved sensor info (hard-coded, not "
                        "enough values):\n";
                    err_output_ << info.as_string();
                }
            }
            else
            {
                // Need to calculate stuff differently since it gave us an
                // aperture value
                info.resolution = DTOR(static_cast<double>(aperture)) /
                    static_cast<double>(info.last_step - info.first_step);
                // Assume that the range is evenly spread
                info.scanable_steps = info.last_step - info.first_step + 1;
                info.front_step = info.scanable_steps / 2 +
                    info.first_step - 1;
                info.min_angle = (static_cast<int>(info.first_step) -
                        static_cast<int>(info.front_step)) * info.resolution;
                info.max_angle = (info.last_step - info.front_step) *
                    info.resolution;

                if(verbose_)
                {
                    err_output_ << "Retrieved sensor info (from FIRM line):\n";
                    err_output_ << info.as_string();
                }
            }
        }
        else
        {
            // We're stuck with hard-coded defaults from the manual (already
            // set earlier).
            info.calculate_values();
            if(verbose_)
            {
                err_output_ << "Retrieved sensor info (hard-coded):\n";
                err_output_ << info.as_string();
            }
        }
    }
    else if(scip_version_ == 2)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Getting sensor information using SCIP version 2.\n";
        }

        info.set_defaults();

        char buffer[SCIP2_LINE_LENGTH];
        memset(buffer, 0, sizeof(char) * SCIP2_LINE_LENGTH);

        // We need to send three commands to get all the info we want: VV, PP
        // and II
        send_command("VV", 0, 0, 0);
        while(read_line_with_check(buffer, -1, true) != 0)
            process_vv_line(buffer, info);

        // Next up, PP
        send_command("PP", 0, 0, 0);
        while(read_line_with_check(buffer, -1, true) != 0)
            process_pp_line(buffer, info);

        // Command II: Revenge of the Commands.
        send_command("II", 0, 0, 0);
        while(read_line_with_check(buffer, -1, true) != 0)
            process_ii_line(buffer, info);

        enable_checksum_workaround_ = false;

        info.calculate_values();
        time_resolution_ = info.time_resolution;
        if(verbose_)
        {
            err_output_ << "Retrieved sensor info:\n";
            err_output_ << info.as_string();
        }
    }
    else
        throw UnknownScipVersionError();
}


unsigned int Sensor::get_time()
{
    return offset_timestamp(wrap_timestamp(get_raw_time()));
}


unsigned int Sensor::get_raw_time()
{
    if(scip_version_ == 1)
        throw UnsupportedError(12);
    else if(scip_version_ == 2)
    {
        if(verbose_)
            err_output_ << "Sensor::" << __func__ <<
                "() Retrieving time from laser.\n";
        send_command("TM", "0", 1, 0);
        send_command("TM", "1", 1, 0);
        char buffer[7];
        read_line_with_check(buffer, 6);
        send_command("TM", "2", 1, 0);
        skip_lines(1);
        // We need to decode the time value that's in the buffer
        return decode_4_byte_value(buffer);
    }
    else
        throw UnknownScipVersionError();

    return 0;
}


long long Sensor::calibrate_time(unsigned int skew_sleep_time,
        unsigned int samples)
{
    if(verbose_)
    {
        err_output_ << "Entering timing mode; start system time is " <<
            get_computer_time() << "ns.\n";
    }
    enter_timing_mode();

    // From A. Carballo, Y. Hara, H. Kawata, T.
    // Yoshida, A. Ohya, S. Yuta, “Time synchronisation
    // between SOKUIKI sensor and host computer using
    // timestamps”, Proceedings of the JSME Conference on
    // Robotics and Mechatronics ROBOMEC 2007, Akita,
    // Japan, 2007, Paper 1P1-K05.
    //
    // Calibration is performed by calculating the offset between the laser's
    // clock and the computer's clock. The algorithm is:
    // Offset = Comp time before -
    //      (Laser time - (Comp time before - Comp time after) / 2)
    // This is calculated samples times, and the median taken.
    if(verbose_)
        err_output_ << "Gathering " << samples << " offset values.\n";
    std::vector<long long> offsets;
    for(unsigned int ii = 0; ii < samples; ii++)
    {
        unsigned long long end_time(0);
        unsigned long long start_time = get_computer_time();
        unsigned long long laser_time =
            wrap_timestamp(get_timing_mode_time(&end_time)) * 1e6;
        offsets.push_back(start_time -
                (laser_time - (end_time - start_time) / 2));
        if(verbose_)
        {
            err_output_ << "Offset #" << ii << " start_time = " << start_time;
            err_output_ << "\tend_time = " << end_time;
            err_output_ << "\tlaser_time = " << laser_time;
            err_output_ << "\tCalculated offset = " <<
                (start_time - (laser_time - (end_time - start_time) / 2)) <<
                '\n';
        }
    }
    // Calculate the median offset
    time_offset_ = median(offsets);
    if(verbose_)
        err_output_ << "Calculated offset is " << time_offset_ << '\n';

    if(skew_sleep_time > 0)
    {
        // Sleep, then do it again to approximate a skew line
        struct timespec sleep_time = {0, 0};
        sleep_time.tv_sec = skew_sleep_time;
        if(verbose_)
            err_output_ << "Sleeping for " << skew_sleep_time << "s.\n";
        nanosleep(&sleep_time, NULL);

        if(verbose_)
            err_output_ << "Gathering " << samples << " offset values.\n";
        offsets.clear();
        for(unsigned int ii = 0; ii < samples; ii++)
        {
            unsigned long long end_time(0);
            unsigned long long start_time = get_computer_time();
            unsigned long long laser_time =
                wrap_timestamp(get_timing_mode_time(&end_time)) * 1e6;
            offsets.push_back(start_time -
                    (laser_time - (end_time - start_time) / 2));
        }
        // Calculate the median offset
        long long offset2 = median(offsets);
        if(verbose_)
        {
            err_output_ << "Calculated second offset is " << offset2 <<
                '\n';
        }

        // Approximate the line slope as (offset2 - offset1) / sleep_time
        time_skew_alpha_ = (offset2 - time_offset_) /
            static_cast<double>(skew_sleep_time * 1e9);
        if(verbose_)
        {
            err_output_ << "Calculated alpha is " << time_skew_alpha_ << '\n';
        }
    }

    // All done.
    if(verbose_)
        err_output_ << "Leaving timing mode.\n";
    leave_timing_mode();
    return time_offset_;
}


unsigned int Sensor::get_ranges(ScanData& data, int start_step,
        int end_step, unsigned int cluster_count)
{
    char buffer[11];
    memset(buffer, 0, sizeof(char) * 11);

    if(start_step < 0)
        start_step = first_step_;
    if(end_step < 0)
        end_step = last_step_;

    unsigned int num_steps = (end_step - start_step + 1) / cluster_count;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " ranges between " << start_step << " and " <<
            end_step << " with a cluster count of " << cluster_count <<
            '\n';
    }

    if(scip_version_ == 1)
    {
        // Send the command to ask for the most recent range data from
        // start_step to end_step
        number_to_string(start_step, buffer, 3);
        number_to_string(end_step, &buffer[3], 3);
        number_to_string(cluster_count, &buffer[6], 2);
        send_command("G", buffer, 8, 0);
        // In SCIP1 mode we're going to get back 2-byte data
        read_2_byte_range_data(data, num_steps);
    }
    else if(scip_version_ == 2)
    {
        // Send the command to ask for the most recent range data from
        // start_step to end_step
        number_to_string(start_step, buffer, 4);
        number_to_string(end_step, &buffer[4], 4);
        number_to_string(cluster_count, &buffer[8], 2);
        if(model_ == MODEL_UXM30LXE && multiecho_mode_ != ME_OFF)
            send_command("HD", buffer, 10, 0);
        else
            send_command("GD", buffer, 10, 0);
        // There will be a timestamp before the data (if there is data)
        // Normally we would send 6 for the expected length, but we may get no
        // timestamp back if there was no data.
        if(read_line_with_check(buffer) == 0)
            throw NoDataError();
        data.laser_time_ = decode_4_byte_value(buffer) +
            step_to_time_offset(start_step);
        data.system_time_ = offset_timestamp(wrap_timestamp(data.laser_time_));
        // In SCIP2 mode we're going to get back 3-byte data because we're
        // sending the GD command
        read_3_byte_range_data(data, num_steps);
    }
    else
        throw UnknownScipVersionError();

    return data.ranges_length_;
}


unsigned int Sensor::get_ranges_by_angle(ScanData& data, double start_angle,
        double end_angle, unsigned int cluster_count)
{
    // Calculate the given angles in steps, rounding towards front_step_
    int start_step, end_step;
    start_step = angle_to_step(start_angle);
    end_step = angle_to_step(end_angle);

    // Check the steps are within the allowable range
    if(start_step < first_step_ || start_step > last_step_)
        throw StartStepError();
    if(end_step < first_step_ || end_step > last_step_)
        throw EndStepError();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Start angle " <<
            start_angle << " is step " << start_step << ", end angle " <<
            end_angle << " is step " << end_step << '\n';
    }

    // Get the data
    return get_ranges(data, start_step, end_step, cluster_count);
}


unsigned int Sensor::get_ranges_intensities(ScanData& data, int start_step,
        int end_step, unsigned int cluster_count)
{
    char buffer[11];
    memset(buffer, 0, sizeof(char) * 11);

    if(start_step < 0)
        start_step = first_step_;
    if(end_step < 0)
        end_step = last_step_;

    unsigned int num_steps = (end_step - start_step + 1) / cluster_count;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " ranges between " << start_step << " and " <<
            end_step << " with a cluster count of " << cluster_count <<
            '\n';
    }

    if(scip_version_ == 1)
        throw UnsupportedError(36);
    else if(scip_version_ != 2)
        throw UnknownScipVersionError();

    // Send the command to ask for the most recent data from
    // start_step to end_step
    number_to_string(start_step, buffer, 4);
    number_to_string(end_step, &buffer[4], 4);
    number_to_string(cluster_count, &buffer[8], 2);
    if(model_ == MODEL_UXM30LXE && multiecho_mode_ != ME_OFF)
        send_command("HE", buffer, 10, 0);
    else
        send_command("GE", buffer, 10, 0);
    // There will be a timestamp before the data (if there is data)
    // Normally we would send 6 for the expected length, but we may get no
    // timestamp back if there was no data.
    if(read_line_with_check(buffer) == 0)
        throw NoDataError();
    data.laser_time_ = decode_4_byte_value(buffer) +
        step_to_time_offset(start_step);
    data.system_time_ = offset_timestamp(wrap_timestamp(data.laser_time_));
    // In SCIP2 mode we're going to get back 3-byte data because we're
    // sending the GE command
    read_3_byte_range_data(data, num_steps);

    return data.ranges_length_;
}


unsigned int Sensor::get_ranges_intensities_by_angle(ScanData& data,
        double start_angle, double end_angle, unsigned int cluster_count)
{
    // Calculate the given angles in steps, rounding towards front_step_
    int start_step, end_step;
    start_step = angle_to_step(start_angle);
    end_step = angle_to_step(end_angle);

    // Check the steps are within the allowable range
    if(start_step < first_step_ || start_step > last_step_)
        throw StartStepError();
    if(end_step < first_step_ || end_step > last_step_)
        throw EndStepError();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Start angle " <<
            start_angle << " is step " << start_step << ", end angle " <<
            end_angle << " is step " << end_step << '\n';
    }

    // Get the data
    return get_ranges_intensities(data, start_step, end_step, cluster_count);
}


unsigned int Sensor::get_new_ranges(ScanData& data, int start_step,
        int end_step, unsigned int cluster_count)
{
    if(scip_version_ == 1)
        throw UnsupportedError(16);
    else if(scip_version_ != 2)
        throw UnknownScipVersionError();

    char buffer[14];
    memset(buffer, 0, sizeof(char) * 14);

    if(start_step < 0)
        start_step = first_step_;
    if(end_step < 0)
        end_step = last_step_;

    unsigned int num_steps = (end_step - start_step + 1) / cluster_count;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " new ranges between " << start_step << " and " <<
            end_step << " with a cluster count of " << cluster_count <<
            '\n';
    }

    // Send the command to ask for the most recent range data from start_step to end_step
    number_to_string(start_step, buffer, 4);
    number_to_string(end_step, &buffer[4], 4);
    number_to_string(cluster_count, &buffer[8], 2);
    number_to_string(1, &buffer[10], 1);
    number_to_string(1, &buffer[11], 2);
    char command[3];
    if(model_ == MODEL_UXM30LXE && multiecho_mode_ != ME_OFF)
        command[0] = 'N';
    else
        command[0] = 'M';
    command[1] = 'D';
    command[2] = '\0';
    send_command(command, buffer, 13, 0);
    // Mx commands will perform a scan, then send the data prefixed with
    // another command echo.
    // Read back the command echo (minimum of 3 bytes, maximum of 16 bytes)
    char response[17];
    skip_lines(1); // End of the command echo message
    read_line(response, 16); // Size is command(2)+params(13)+new line(1)
    // Check the echo is correct
    if(response[0] != command[0] || response[1] != command[1])
        throw CommandEchoError(command, response);
    // Then compare the parameters
    buffer[12] = '0'; // There will be zero scans remaining after this one
    if(memcmp(&response[2], buffer, 13) != 0)
        throw ParamEchoError(command);
    // The next line should be the status line
    read_line_with_check(response, 4);
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() " << command << " data prefix status: " << response[0] <<
            response[1] << '\n';
    }
    // Check the status code is OK - should only get 99 here
    if(response[0] != '9' || response[1] != '9')
    {
        // There is an extra line feed after an error status (signalling
        // end of message)
        skip_lines(1);
        throw ResponseError(response, command);
    }

    // Now the actual data will arrive
    // There will be a timestamp before the data (if there is data)
    // Normally we would send 6 for the expected length, but we may get no
    // timestamp back if there was no data.
    if(read_line_with_check(buffer) == 0)
        throw NoDataError();
    data.laser_time_ = decode_4_byte_value(buffer) +
        step_to_time_offset(start_step);
    data.system_time_ = offset_timestamp(wrap_timestamp(data.laser_time_));
    // In SCIP2 mode we're going to get back 3-byte data because we're
    // sending the MD command
    read_3_byte_range_data(data, num_steps);

    return data.ranges_length_;
}


unsigned int Sensor::get_new_ranges_by_angle(ScanData& data,
        double start_angle, double end_angle, unsigned int cluster_count)
{
    if(scip_version_ == 1)
        throw UnsupportedError(16);

    // Calculate the given angles in steps, rounding towards front_step_
    int start_step, end_step;
    start_step = angle_to_step(start_angle);
    end_step = angle_to_step(end_angle);

    // Check the steps are within the allowable range
    if(start_step < first_step_ || start_step > last_step_)
        throw StartStepError();
    if(end_step < first_step_ || end_step > last_step_)
        throw EndStepError();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Start angle " <<
            start_angle << " is step " << start_step << ", end angle " <<
            end_angle << " is step " << end_step << '\n';
    }

    // Get the data
    return get_new_ranges(data, start_step, end_step, cluster_count);
}


unsigned int Sensor::get_new_ranges_intensities(ScanData& data,
        int start_step, int end_step, unsigned int cluster_count)
{
    if(scip_version_ == 1)
        throw UnsupportedError(17);
    else if(scip_version_ != 2)
        throw UnknownScipVersionError();

    char buffer[14];
    memset(buffer, 0, sizeof(char) * 14);

    if(start_step < 0)
        start_step = first_step_;
    if(end_step < 0)
        end_step = last_step_;

    unsigned int num_steps = (end_step - start_step + 1) / cluster_count;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " new ranges and intensities between " <<
            start_step << " and " << end_step <<
            " with a cluster count of " << cluster_count << '\n';
    }

    // Send the command to ask for the most recent range data with
    // intensity data from start_step to end_step
    number_to_string(start_step, buffer, 4);
    number_to_string(end_step, &buffer[4], 4);
    number_to_string(cluster_count, &buffer[8], 2);
    number_to_string(1, &buffer[10], 1);
    number_to_string(1, &buffer[11], 2);
    char command[3];
    if(model_ == MODEL_UXM30LXE && multiecho_mode_ != ME_OFF)
        command[0] = 'N';
    else
        command[0] = 'M';
    command[1] = 'E';
    command[2] = '\0';
    send_command(command, buffer, 13, 0);
    // Mx commands will perform a scan, then send the data prefixed with
    // another command echo.
    // Read back the command echo (minimum of 3 bytes, maximum of 16 bytes)
    char response[17];
    skip_lines(1); // End of the command echo message
    read_line(response, 16); // Size is command(2)+params(13)+new line(1)
    // Check the echo is correct
    if(response[0] != command[0] || response[1] != command[1])
        throw CommandEchoError(command, response);
    // Then compare the parameters
    buffer[12] = '0'; // There will be zero scans remaining after this one
    if(memcmp(&response[2], buffer, 13) != 0)
        throw ParamEchoError(command);
    // The next line should be the status line
    read_line_with_check(response, 4);
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() " << command << " data prefix status: " << response[0] <<
            response[1] << '\n';
    }
    // Check the status code is OK - should only get 99 here
    if(response[0] != '9' || response[1] != '9')
    {
        // There is an extra line feed after an error status (signalling
        // end of message)
        skip_lines(1);
        throw ResponseError(response, command);
    }

    // Now the actual data will arrive
    // There will be a timestamp before the data (if there is data)
    // Normally we would send 6 for the expected length, but we may get no
    // timestamp back if there was no data.
    if(read_line_with_check(buffer) == 0)
        throw NoDataError();
    data.laser_time_ = decode_4_byte_value(buffer) +
        step_to_time_offset(start_step);
    data.system_time_ = offset_timestamp(wrap_timestamp(data.laser_time_));
    // In SCIP2 mode we're going to get back 3-byte data because we're
    // sending the ME command
    read_3_byte_range_and_intensity_data(data, num_steps);

    return data.ranges_length_;
}


unsigned int Sensor::get_new_ranges_intensities_by_angle(ScanData& data,
        double start_angle, double end_angle, unsigned int cluster_count)
{
    if(scip_version_ == 1)
        throw UnsupportedError(17);

    // Calculate the given angles in steps, rounding towards front_step_
    int start_step, end_step;
    start_step = angle_to_step(start_angle);
    end_step = angle_to_step(end_angle);

    // Check the steps are within the allowable range
    if(start_step < first_step_ || start_step > last_step_)
        throw StartStepError();
    if(end_step < first_step_ || end_step > last_step_)
        throw EndStepError();

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Start angle " <<
            start_angle << " is step " << start_step << ", end angle " <<
            end_angle << " is step " << end_step << '\n';
    }

    // Get the data
    return get_new_ranges_intensities(data, start_step, end_step,
            cluster_count);
}


double Sensor::step_to_angle(unsigned int step)
{
    return (static_cast<int>(step) - static_cast<int>(front_step_)) *
        resolution_;
}


unsigned int Sensor::angle_to_step(double angle)
{
    unsigned int result;
    double resultF;
    resultF = front_step_ +
        (static_cast<double>(angle) / static_cast<double>(resolution_));
    // Round towards front_step_ so that the step values are always inside the
    // angles given
    if(resultF < front_step_)
        result = static_cast<int>(ceil(resultF));
    else
        result = static_cast<int>(floor(resultF));

    return result;
}


// Private functions
///////////////////////////////////////////////////////////////////////////////

// Sometimes, just flushing isn't enough, as it appears the scanner will wait
// when the buffer gets full, then continue sending data. If we flush, we get
// rid of what was sent, and the scanner just sends more. This is a problem if
// we're trying to clear the result of a previous command.  To get around this,
// keep flushing until the port reports there is no data left after a timeout.
// This shouldn't be called too much, as it introduces a delay as big as the
// timeout (which may be infinite).
void Sensor::clear_read_buffer()
{
    while(port_->BytesAvailableWait() > 0)
        port_->Flush();
}


// If expected_length is not -1, it should include the terminating line feed but
// not the 0 (although the buffer still has to include this).
// If expected_length is -1, this function expects buffer to be a certain length
// to allow up to the maximum line length to be read. See SCIP1_LINE_LENGTH and
// SCIP2_LINE_LENGTHr
// If fast is true, Read() will be called instead of ReadLine(), which should
// result in a faster, less CPU-intensive read. This is used, for example, when
// reading the range data.
// The line feed that terminates a line will be replaced with a 0.
// The return value is the number of bytes received, not including the 0
// byte or the line feed.
int Sensor::read_line(char* buffer, int expected_length)
{
    int linelength = 0;

    if(expected_length == -1)
    {
        int maxlength = (scip_version_ == 1) ?
            SCIP1_LINE_LENGTH : SCIP2_LINE_LENGTH;
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ << "() Reading up to " <<
                maxlength << " bytes.\n";
        }
        // We need to get at least 1 byte in a line: the line feed.
        if((linelength = port_->ReadLine(buffer, maxlength)) < 0)
            throw ReadError(0);
        else if(linelength == 0)
            throw ReadError(1);
        // Replace the line feed with a 0
        buffer[linelength - 1] = '\0';
    }
    else
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Reading exactly " << expected_length << " bytes.\n";
        }
        // expected_length+1 for the 0
        if((linelength = port_->ReadLine(buffer, expected_length + 1)) < 0)
            throw ReadError(0);
        else if(linelength == 0)
            throw ReadError(1);
        else if(linelength < expected_length)
            throw LineLengthError(linelength, expected_length);
        // Replace the line feed with a 0
        buffer[linelength - 1] = '\0';
    }

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Read " << linelength <<
            " bytes.\n";
        err_output_ << "Sensor::" << __func__ << "() Line is '" << buffer <<
            "'\n";
    }
    return linelength - 1; // Line feed not included
}


// This function will read a line and then calculate its checksum, comparing it
// with the checksum at the end of the line. The checksum will be removed
// (along with the semi-colon, if present).  buffer and expected_length args are
// as for read_line().
// If has_semicolon is true, the byte before the checksum is assumed to be the
// semi-colon separator and so not a part of the checksum. If it's not a
// semi-colon, an exception is thrown.
// Empty lines (i.e. a line that is just the line feed, as at the end of the
// message) will result in a return value of zero and no checksum check will be
// performed. Otherwise the number of actual data bytes (i.e. excluding the
// checksum and semicolon) will be returned.
// BIG NOTE: The UTM-30LX has a probable bug in its response to the II code
// whereby it does not include anything after and including "<-" in the
// checksum calculation. A workaround is enabled in this function when
// model_ is MODEL_UTM30LX. In this case, if the checksum fails normally, it
// scans the line for "<-" and recalculates the checksum on the bytes up to
// that point. This only happens in SCIP v2.
int Sensor::read_line_with_check(char* buffer, int expected_length,
        bool has_semicolon)
{
    int linelength = read_line(buffer, expected_length);
    if(scip_version_ == 1)
    {
        // No checksums in SCIP version 1
        return linelength;
    }

    // If the line is empty, assume it was a line-feed message terminator, in
    // which case there is no checksum to check.
    if(linelength == 0)
        return 0;

    // Ignore the checksum itself, and possibly a semicolon (read_line_ has
    // already chopped off the line feed for us).
    int bytesToConsider = linelength - 1 - (has_semicolon ? 1 : 0);
    int checksumIndex = bytesToConsider + (has_semicolon ? 1 : 0);
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Considering " <<
            bytesToConsider << " bytes for checksum from a line length of " <<
            linelength << " bytes.\n";
    }
    if(bytesToConsider < 1)
        throw InsufficientBytesError(bytesToConsider, linelength);

    //DTC:var set but not used
    //int checksum = 0;
    try
    {
        confirm_checksum(buffer, bytesToConsider,
                static_cast<int>(buffer[checksumIndex]));
    }
    catch(ProtocolError& e)
    {
        if(model_ == MODEL_UTM30LX && enable_checksum_workaround_)
        {
            // Here comes the UTM-30LX workaround
            char* hasComment = strstr(buffer, "<-");
            if(hasComment != 0)
            {
                int newBytesToConsider = hasComment - buffer;
                if(verbose_)
                {
                    err_output_ << "Sensor::" << __func__ <<
                        "() Performing UTM-30LX II response "
                        "checksum workaround: trying with " <<
                        newBytesToConsider << " bytes.\n";
                }
                if(newBytesToConsider < 1)
                {
                    throw InsufficientBytesError(newBytesToConsider,
                            linelength);
                }

                confirm_checksum(buffer, newBytesToConsider,
                        static_cast<int>(buffer[checksumIndex]));
            }
            else
                // Workaround is disabled - rethrow
                throw;
        }
        else
            // Not a workaround-compatible error - rethrow
            throw;
    }

    // Null out the semi-colon (if there) and checksum
    buffer[bytesToConsider] = '\0';

    return bytesToConsider;
}


// Data blocks are treated as a special case of lines. This allows us to easily
// implement a faster read with less condition checks, as the format is more
// uniform.
// Returns true if the read data block was not a full data block, and thus is
// the end of the data.
bool Sensor::read_data_block(char* buffer, int& block_size)
{
    bool is_last(false);

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading exactly " <<
            DATA_BLOCK_LENGTH + 2 << " bytes.\n";
    }
    // Read up to DATA_BLOCK_SIZE + 1 (checksum) + 1 (line feed) bytes,
    // stopping as soon as a read puts a new line on the end
    block_size = 0;
    int read_goal = DATA_BLOCK_LENGTH + 2;
    while (block_size < read_goal)
    {
        int bytes_read(0);
        if((bytes_read = port_->Read(&buffer[block_size],
                        read_goal - block_size)) < 0)
        {
            throw ReadError(0);
        }
        else if(bytes_read == 0)
        {
            throw ReadError(1);
        }
        block_size += bytes_read;
        if (buffer[block_size - 1] == '\n')
        {
            // New line = end of data line
            break;
        }
    }
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Read " << block_size <<
            " bytes.\n";
        err_output_ << "Sensor::" << __func__ << "() Line is '" << buffer <<
            "'\n";
    }
    // The data block should finish with one or two new lines. If it finishes
    // with two, this is the last data block.
    if (buffer[block_size - 1] != '\n')
    {
        throw ReadError(38);
    }
    buffer[block_size - 1] = '\0';
    block_size -= 1; // Remove the new line
    if (buffer[block_size - 1] == '\n')
    {
        if (verbose_)
        {
            err_output_ << "Sensor::" << __func__ << "() Found last block.\n";
        }
        is_last = true;
        buffer[block_size - 1] = '\0';
        block_size -= 1; // So it doesn't get used in the checksum calculation
    }
    // Check the checksum, which is the last byte in the block.
    int bytes_to_consider = block_size - 1;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Considering " <<
            bytes_to_consider << " bytes for checksum from a line length of " <<
            block_size << " bytes.\n";
    }
    if(bytes_to_consider < 1)
        throw InsufficientBytesError(bytes_to_consider, block_size);
    confirm_checksum(buffer, bytes_to_consider,
            static_cast<int>(buffer[bytes_to_consider]));
    // Nullify the checksum
    buffer[bytes_to_consider] = '\0';
    block_size -= 1;

    return is_last;
}


// Reads lines until the number specified has passed.
void Sensor::skip_lines(int count)
{
    if(verbose_)
        err_output_ << "Sensor::" << __func__ << "() Skipping " << count <<
            " lines.\n";
    if(port_->SkipUntil(0x0A, count) < 0)
        throw ReadError(18);
}


// Sends a command with optional parameters and checks that the echo of the
// command and parameters sent are correct, and that the returned status code
// is 0 or the first byte of extra_ok (for SCIP1), or 00, 99 or the first two
// bytes of extra_ok (for SCIP2).
// cmd must be a 1 byte string for SCIP1 and a 2-byte 0-terminated string
// for SCIP2.
// If param_length is 0, no parameters will be sent or expected in the reply.
// extra_ok must be a 1-byte string for SCIP1 and a 2-byte string for SCIP2.
// Return value is the status code returned for the command.
int Sensor::send_command(char const* cmd, char const* param,
        int param_length, char const* extra_ok)
{
    int statusCode = -1;
    char response[17];

    // Flush first to clear out the dregs of any previous commands
    port_->Flush();

    if(scip_version_ == 1)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Writing in SCIP1 mode. Command is " << cmd[0] <<
                ", parameters length is " << param_length << '\n';
        }
        // Write the command
        if(port_->Write(cmd, 1) < 1)
            throw WriteError(19);
        if(param_length > 0)
        {
            if(port_->Write(param, param_length) < param_length)
                throw WriteError(20);
        }
        if(port_->Write("\n", 1) < 1)
            throw WriteError(21);

        // Read back the response (should get at least 4 bytes , possibly up to
        // 16 including \n's depending on the parameters): cmd[0] params \n
        // status \n
        int statusIndex = 2 + param_length;
        read_line(response, 2 + param_length);
        read_line(&response[statusIndex], 2);
        // First make sure that the echoed command matches
        if(response[0] != cmd[0])
        {
            char temp_cmd[2];
            temp_cmd[0] = cmd[0];
            temp_cmd[1] = '\0';
            char temp_echo[2];
            temp_echo[0] = response[0];
            temp_echo[1] = '\0';
            throw CommandEchoError(temp_cmd, temp_echo);
        }
        // Then compare the parameters
        if(param_length > 0)
        {
            if(memcmp(&response[1], param, param_length) != 0)
            {
                char temp_cmd[2];
                temp_cmd[0] = cmd[0];
                temp_cmd[1] = '\0';
                throw ParamEchoError(temp_cmd);
            }
        }
        // Next up, check the status byte
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Command response status: " << response[statusIndex] <<
                '\n';
        }
        if(response[statusIndex] != '0')
        {
            if(extra_ok != 0)
            {
                if(response[statusIndex] != extra_ok[0])
                {
                    // There is an extra line feed after an error status
                    // (signalling end of message)
                    skip_lines(1);
                    throw Scip1ResponseError(response[statusIndex], cmd[0]);
                }
            }
            else
            {
                // There is an extra line feed after an error status
                // (signalling end of message)
                skip_lines(1);
                throw Scip1ResponseError(response[statusIndex], cmd[0]);
            }
        }
        statusCode = atoi(&response[statusIndex]);
        // All OK, data starts at beginning of port's buffer
    }
    else if(scip_version_ == 2)
    {
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Writing in SCIP2 mode. Command is " << cmd <<
                ", parameters length is " << param_length << '\n';
        }
        // Write the command
        if(port_->Write(cmd, 2) < 2)
            throw WriteError(19);
        if(param_length > 0)
        {
            if(port_->Write(param, param_length) < param_length)
                throw WriteError(20);
        }
        if(port_->Write("\n", 1) < 1)
            throw WriteError(21);

        // Read back the command echo (minimum of 3 bytes, maximum of 16 bytes)
        read_line(response, 3 + param_length);
        // Check the echo is correct
        if(response[0] != cmd[0] || response[1] != cmd[1])
            throw CommandEchoError(cmd, response);
        // Then compare the parameters
        if(param_length > 0)
        {
            if(memcmp(&response[2], param, param_length) != 0)
                throw ParamEchoError(cmd);
        }

        // The next line should be the status line
        read_line_with_check(response, 4);
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Command response status: " << response[0] << response[1] <<
                '\n';
        }
        // Check the status code is OK
        response[2] = '\0';
        if(!(response[0] == '0' && response[1] == '0') &&
            !(response[0] == '9' && response[1] == '9'))
        {
            if(extra_ok != 0)
            {
                if(response[0] != extra_ok[0] || response[1] != extra_ok[1])
                {
                    // There is an extra line feed after an error status
                    // (signalling end of message)
                    skip_lines(1);
                    throw ResponseError(response, cmd);
                }
            }
            else
            {
                // There is an extra line feed after an error status
                // (signalling end of message)
                skip_lines(1);
                throw ResponseError(response, cmd);
            }
        }
        statusCode = atoi(response);
        // All OK, data starts at beginning of port's buffer
    }
    else
        throw UnknownScipVersionError();

    return statusCode;
}


/// Puts the laser into the timing mode.
void Sensor::enter_timing_mode()
{
    send_command("TM", "0", 1, 0);
    skip_lines(1);
}


/// Take the laser out of timing mode.
void Sensor::leave_timing_mode()
{
    send_command("TM", "2", 1, 0);
    skip_lines(1);
}


/// Get the timestamp from the laser.
/// If @ref reception_time is not 0, it will be filled with the time at
/// which data reception was completed.
unsigned int Sensor::get_timing_mode_time(unsigned long long* reception_time)
{
    // To get the most accurate result, we cannot do any processing while
    // receiving. We want to know the time that reception finished as exactly
    // as possible. To achieve this, we do not use send_command_, but instead
    // send the command manually and receive the entire expected reply at once,
    // then check/decode it later.
    char response[17];
    if(port_->Write("TM1\n", 4) < 4)
        throw WriteError(19);
    unsigned int line_length = port_->Read(response, 16);
    if(reception_time)
        *reception_time = get_computer_time();
    // Process the response to confirm it is correct
    if(line_length < 0)
        throw ReadError(0);
    else if(line_length == 0)
        throw ReadError(1);
    else if(line_length < 15)
        throw LineLengthError(line_length, 15);
    response[line_length - 1] = '\0';
    if(response[0] != 'T' || response[1] != 'M' || response[2] != '1')
        throw CommandEchoError("TM", response);
    response[7] = '\0';
    if(response[4] != '0' || response[5] != '0' || response[6] != 'P')
        throw ResponseError(response, "TM");
    // Check the checksum on the time stamp is accurate
    confirm_checksum(&response[8], 4, response[12]);
    // Decode the time stamp
    unsigned int timestamp =
        decode_4_byte_value(&response[8]);
    return timestamp;
}


/// Get the computer's time as accurately as possible.
unsigned long long Sensor::get_computer_time()
{
#if defined(_POSIX_TIMERS)
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1e9 + ts.tv_nsec;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec * 1e9 + tv.tv_usec * 1e3;
#endif
}


/// timestamp must be in milliseconds. The result is in milliseconds.
unsigned int Sensor::wrap_timestamp(unsigned int timestamp)
{
    if(timestamp < last_timestamp_)
    {
        wrap_count_++;
    }
    last_timestamp_ = timestamp;

    return timestamp + wrap_count_ * 0x01000000; // 24-bit value + 1
}


/// timestamp must be in milliseconds. The result is in nanoseconds.
unsigned long long Sensor::offset_timestamp(unsigned int timestamp)
{
    return ((1 - time_drift_rate_) * timestamp * 1e6 + time_offset_) /
        (1 - time_skew_alpha_);
}


unsigned int Sensor::step_to_time_offset(int start_step)
{
    if(start_step < 0)
        return first_step_ * time_resolution_;
    else
        return start_step * time_resolution_;
}


/// Search a string for the laser's model.
void Sensor::find_model(char const* buffer)
{
    if(strstr(buffer, "URG-04LX") != 0)
        model_ = MODEL_URG04LX;
    else if(strstr(buffer, "UBG-04LX") != 0)
        model_ = MODEL_UBG04LXF01;
    else if(strstr(buffer, "UHG-08LX") != 0)
        model_ = MODEL_UHG08LX;
    else if(strstr(buffer, "UTM-30LX") != 0)
    {
        model_ = MODEL_UTM30LX;
        // Also enable the work around for a checksum problem in this
        // model.
        enable_checksum_workaround_ = true;
    }
    else if(strstr(buffer, "UXM-30LX") != 0)
        model_ = MODEL_UXM30LXE;
    else
        model_ = MODEL_UNKNOWN;
}


void Sensor::get_and_set_scip_version()
{
    bool scip2Failed = false;


    if(verbose_)
        err_output_ << "Sensor::" << __func__ <<
            "() Testing SCIP protocol version.\n";
    // Try SCIP version 2 first by sending an info command
    try
    {
        send_command("VV", 0, 0, 0);
    }
    catch(BaseError)
    {
        // That didn't work too well...
        if(verbose_)
            err_output_ << "Sensor::" << __func__ <<
                "() Initial SCIP version 2 test failed.\n";
        scip2Failed = true;
    }

    if(scip2Failed)
    {
        // Currently using SCIP version 1
        // Get the firmware version and check if we can move to SCIP version 2
        scip_version_ = 1;

        port_->Flush();
        try
        {
            send_command("V", 0, 0, 0);
        }
        catch(BaseError)
        {
            throw ScipVersionError();
        }
        // Skip the vendor and product info
        skip_lines(2);
        // Get the firmware line
        char buffer[SCIP1_LINE_LENGTH];
        memset(buffer, 0, sizeof(char) * SCIP1_LINE_LENGTH);
        read_line(buffer);

        if(strncmp(buffer, "FIRM:", 5) != 0)
            throw MissingFirmSpecError();
        // Pull out the major version number
        // Note that although lasers such as the UTM-30LX appear to use a
        // different firmware version format, that doesn't matter because they
        // don't support SCIP v1 and so shouldn't get to this point anyway - if
        // they do, it's an uncaught error.
        int majorVer = strtol(&buffer[5], 0, 10);
        if(errno == ERANGE)
            throw FirmwareError();
        if(verbose_)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Firmware major version is " << majorVer <<
                '\n';
        }
        // Dump the rest of the V command result (one of these will be the
        // empty last line)
        skip_lines(3);

        // If the firmware version is less than 3, we're stuck with SCIP
        // version 1.
        if(majorVer < 3)
        {
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Firmware does not support SCIP version 2; using SCIP "
                    "version 1.\n";
            return;
        }
        // Otherwise we can try SCIP version 2
        else
        {
            port_->Flush();
            // We'll hijack the send_command_ function a bit here. Normally it
            // takes 1-byte commands, (we're currently using SCIP version 1,
            // remember), but the command to change to SCIP version 2 is 7
            // bytes long (why did they have to do it that way?). So send the
            // first byte as the command and the other 6 as parameters.
            try
            {
                send_command("S", "CIP2.0", 6, 0);
            }
            catch(BaseError)
            {
                if(verbose_)
                    err_output_ << "Sensor::" << __func__ <<
                        "() Could not change to SCIP version 2; using SCIP "
                        "version 1.\n";
                return;
            }
            // There'll be a trailing line on the end
            skip_lines(1);

            // Changed to SCIP version 2
            if(verbose_)
                err_output_ << "Sensor::" << __func__ <<
                    "() Using SCIP version 2.\n";
            scip_version_ = 2;
            return;
        }
    }
    else
    {
        // Currently using SCIP version 2
        scip_version_ = 2;

        // Dump the rest of the result
        skip_lines(6);
        if(verbose_)
            err_output_ << "Sensor::" << __func__ <<
                "() Using SCIP version 2.\n";
        return;
    }

    // Fallback case if didn't find a good SCIP version and return above
    throw ScipVersionError();
}


void Sensor::get_defaults()
{
    if(verbose_)
        err_output_ << "Sensor::" << __func__ <<
            "() Getting default values.\n";

    // Get the laser's info
    SensorInfo info;
    get_sensor_info(info);

    min_angle_ = info.min_angle;
    max_angle_ = info.max_angle;
    resolution_ = info.resolution;
    first_step_ = info.first_step;
    last_step_ = info.last_step;
    front_step_ = info.front_step;
    max_range_ = info.max_range;
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() Got default values: " << min_angle_ << " " << max_angle_ <<
            " " << resolution_ << " " << first_step_ << " " << last_step_ <<
            " " << front_step_ << " " << max_range_ << '\n';
    }
}


void Sensor::process_vv_line(char const* buffer, SensorInfo& info)
{
    if(strncmp(buffer, "VEND", 4) == 0)
        info.vendor = &buffer[5]; // Vendor info, minus the "VEND:" tag
    else if(strncmp(buffer, "PROD", 4) == 0)
    {
        info.product = &buffer[5]; // Product info
        // Find the product model
        find_model(&buffer[5]);
        info.detected_model = model_;
    }
    else if(strncmp(buffer, "FIRM", 4) == 0)
        info.firmware = &buffer[5]; // Firmware version
    else if(strncmp(buffer, "PROT", 4) == 0)
        info.protocol = &buffer[5]; // Protocol version
    else if(strncmp(buffer, "SERI", 4) == 0)
        info.serial = &buffer[5]; // Serial number
    else if(!ignore_unknowns_)
        throw UnknownLineError(buffer);
}


void Sensor::process_pp_line(char const* buffer, SensorInfo& info)
{
    if(strncmp(buffer, "MODL", 4) == 0)
        info.model = &buffer[5];   // Model
    // On to the fun ones that require parsing
    else if(strncmp(buffer, "DMIN", 4) == 0)
        info.min_range = atoi(&buffer[5]);
    else if(strncmp(buffer, "DMAX", 4) == 0)
        info.max_range = atoi(&buffer[5]);
    else if(strncmp(buffer, "ARES", 4) == 0)
        info.steps = atoi(&buffer[5]);
    else if(strncmp(buffer, "AMIN", 4) == 0)
        info.first_step = atoi(&buffer[5]);
    else if(strncmp(buffer, "AMAX", 4) == 0)
        info.last_step = atoi(&buffer[5]);
    else if(strncmp(buffer, "AFRT", 4) == 0)
        info.front_step = atoi(&buffer[5]);
    else if(strncmp(buffer, "SCAN", 4) == 0)
        info.standard_speed = atoi(&buffer[5]);
    /* No example in the manual and sensor with support for this has not
     * arrived yet, so don't know what to look for.
    else if(strncmp(buffer, "", 4) == 0)
    {
        if(strstr(buffer, "CCW") != 0)
            info.rot_dir = COUNTERCLOCKWISE;
        else
            info.rot_dir = CLOCKWISE;
    }*/
    else if(!ignore_unknowns_)
        throw UnknownLineError(buffer);
}


void Sensor::process_ii_line(char const* buffer, SensorInfo& info)
{
    if(strncmp(buffer, "MODL", 4) == 0)
        // Do nothing here - we already know this value from PP
        return;
    else if(strncmp(buffer, "LASR", 4) == 0)
    {
        if(strncmp(&buffer[5], "OFF", 3) == 0)
            info.power = false;
        else
            info.power = true;
    }
    else if(strncmp(buffer, "SCSP", 4) == 0)
    {
        if(strncmp(&buffer[5], "Initial", 7) == 0)
        {
            // Unchanged motor speed
            if(sscanf(buffer, "SCSP:%*7s(%d[rpm]", &info.speed) != 1)
            {
                throw ParseError(buffer, "Motor speed");
            }
            info.speed_level = 0;
        }
        else
        {
            // Changed motor speed, format is:
            // <level>%<ignored string>(<speed>[rpm])
            if(sscanf(buffer, "SCSP:%hd%%%*4s(%d[rpm]", &info.speed_level,
                        &info.speed) != 2)
            {
                throw ParseError(buffer, "Motor speed");
            }
        }
    }
    else if(strncmp(buffer, "MESM", 4) == 0)
        info.measure_state = &buffer[5];
    else if(strncmp(buffer, "SBPS", 4) == 0)
    {
        if(strncmp(&buffer[5], "USB only", 8) == 0 ||
            strncmp(&buffer[5], "USB Full Speed", 14) == 0)
        {
            // No baud rate for USB-only devices such as the UHG-08LX
            info.baud = 0;
        }
        else if(sscanf(buffer, "SBPS:%d[bps]", &info.baud) != 1)
            throw ParseError(buffer, "Baud rate");
    }
    else if(strncmp(buffer, "TIME", 4) == 0)
    {
        if(sscanf(buffer, "TIME:%x", &info.time) != 1)
            throw ParseError(buffer, "Timestamp");
    }
    else if(strncmp(buffer, "STAT", 4) == 0)
        info.sensor_diagnostic = &buffer[5];
    else if(!ignore_unknowns_)
        throw UnknownLineError(buffer);
}


/// Combines up to three values from an echo buffer into a single value based
/// on the setting of multiecho_mode_.
uint32_t Sensor::process_echo_buffer(int const* buffer, int num_echos)
{
    uint32_t sum = 0;
    switch(multiecho_mode_)
    {
        case ME_FRONT:
            return buffer[0];
            break;
        case ME_MIDDLE:
            if(num_echos == 3)
                return buffer[1];
            else
                return buffer[0];
            break;
        case ME_REAR:
            return buffer[num_echos -1 ];
            break;
        case ME_AVERAGE:
            for (int ii = 0; ii < num_echos; ii++)
                sum += buffer[ii];
            sum /= static_cast<float>(num_echos);
            return sum;
            break;
        case ME_OFF:
        default:
            return buffer[0];
            break;
    }
}


void Sensor::read_2_byte_range_data(ScanData& data, unsigned int num_steps)
{
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " ranges.\n";
    }

    // This will automatically take care of whether it actually needs to
    // (re)allocate or not.
    data.allocate_data(num_steps);
    data.model_ = model_;
    data.error_ = false;

    // 2 byte data is easy since it fits neatly in a 64-byte block
    char buffer[SCIP2_LINE_LENGTH];
    unsigned int current_step(0);
    int numBytesInLine(0);
    bool done(false);
    while(!done)
    {
        // Read a line of data
        done = read_data_block(buffer, numBytesInLine);
        // Check if we've reached the end of the data
        if(numBytesInLine == 0)
        {
            err_output_ << "numBytesInLine is zero!\n";
        }
        // Process pairs of bytes until we encounter the end of the line
        for (int ii = 0; ii < numBytesInLine; ii += 2, current_step++)
        {
            if(buffer[ii] == '\n' || buffer[ii + 1] == '\n')
            {
                // Line feed in the middle of a data block? Why?
                throw MisplacedLineFeedError();
            }
            data.write_range(current_step, decode_2_byte_value(&buffer[ii]));
        }
        // End of this line. Go around again.
    }

    if(verbose_)
        err_output_ << "Sensor::" << __func__ << "() Read " <<
            current_step << " ranges.\n";
    if(current_step != num_steps)
        throw DataCountError();
}


void Sensor::read_3_byte_range_data(ScanData& data, unsigned int num_steps)
{
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " ranges.\n";
        if(multiecho_mode_ != ME_OFF)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Multi-echo mode is set to " <<
                multiecho_mode_to_string(multiecho_mode_) << '\n';
        }
    }

    // This will automatically take care of whether it actually needs to
    // (re)allocate or not.
    data.allocate_data(num_steps);
    data.model_ = model_;
    data.error_ = false;

    // 3 byte data is a pain because it crosses the line boundary, it may
    // overlap by 0, 1 or 2 bytes
    char buffer[SCIP2_LINE_LENGTH];
    unsigned int current_step(0);
    int numBytesInLine(0), split_count(0);
    char split_value[3];
    int echo_buffer[3] = {-1, -1, -1};
    int echo_buf_ind(0);
    bool done(false);
    while(!done)
    {
        // Read a line of data
        done = read_data_block(buffer, numBytesInLine);
        // Check if we've reached the end of the data
        if(numBytesInLine == 0)
        {
            err_output_ << "numBytesInLine is zero!\n";
        }
        // Process triplets of bytes until we encounter or overrun the end of
        // the line
        for (int ii = 0; ii < numBytesInLine;)
        {
            if(buffer[ii] == '\n' || buffer[ii + 1] == '\n')
            {
                // Line feed in the middle of a line? Why?
                throw MisplacedLineFeedError();
            }
            if(split_count == 0)
            {
                // Start of a value. Decide where to store the next value, and
                // if the previous is complete.
                if(buffer[ii] == '&')
                {
                    // Next echo
                    echo_buf_ind++;
                    ii++;
                }
                else if(echo_buffer[0] != -1)
                {
                    // Not the first value, so deal with the previous
                    data.write_range(current_step,
                            process_echo_buffer(echo_buffer,
                                echo_buf_ind + 1));
                    if(data.ranges_)
                    {
                        if(data.ranges_[current_step] > max_range_)
                        {
                            err_output_ << "WARNING: Sensor::" << __func__ <<
                                "() Value at step " << current_step <<
                                " beyond maximum range: " <<
                                data.ranges_[current_step] << '\n';
                        }
                    }
                    current_step++;
                    echo_buf_ind = 0;
                    echo_buffer[0] = -1;
                }
            }
            if(ii == numBytesInLine - 2)       // Short 1 byte
            {
                split_value[0] = buffer[ii];
                split_value[1] = buffer[ii + 1];
                // Will be reset on the next iteration, after it's used
                split_count = 1;
                ii += 2;
            }
            else if(ii == numBytesInLine - 1)  // Short 2 bytes
            {
                split_value[0] = buffer[ii];
                // Will be reset on the next iteration, after it's used
                split_count = 2;
                ii += 1;
            }
            else
            {
                if(split_count == 1)
                {
                    split_value[2] = buffer[ii++];
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(split_value);
                }
                else if(split_count == 2)
                {
                    split_value[1] = buffer[ii++];
                    split_value[2] = buffer[ii++];
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(split_value);
                }
                else
                {
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(&buffer[ii]);
                    ii += 3;
                }
                split_count = 0;     // Reset this here now that it's been used
            }
        }
        // End of this line. Go around again.
    }
    // Last little bit of data
    if(echo_buffer[0] != -1)
    {
        // Not the first value, so deal with the previous
        data.write_range(current_step,
                process_echo_buffer(echo_buffer, echo_buf_ind + 1));
        if(data.ranges_)
        {
            if(data.ranges_[current_step] > max_range_)
            {
                err_output_ << "WARNING: Sensor::" << __func__ <<
                    "() Value at step " << current_step <<
                    " beyond maximum range: " <<
                    data.ranges_[current_step] << '\n';
            }
        }
        current_step++;
    }

    if(verbose_)
        err_output_ << "Sensor::" << __func__ << "() Read " <<
            current_step << " ranges.\n";
    if(current_step != num_steps)
        throw DataCountError();
}


void Sensor::read_3_byte_range_and_intensity_data(ScanData& data,
        unsigned int num_steps)
{
    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Reading " <<
            num_steps << " ranges and intensities.\n";
        if(multiecho_mode_ != ME_OFF)
        {
            err_output_ << "Sensor::" << __func__ <<
                "() Multi-echo mode is set to " <<
                multiecho_mode_to_string(multiecho_mode_) << '\n';
        }
    }

    // This will automatically take care of whether it actually needs to
    // (re)allocate or not.
    data.allocate_data(num_steps, true);
    data.model_ = model_;

    // 3 byte data is a pain because it crosses the line boundary, it may
    // overlap by 0, 1 or 2 bytes
    char buffer[SCIP2_LINE_LENGTH];
    unsigned int current_range(0), current_intensity(0);
    int numBytesInLine(0), split_count(0);
    char split_value[3];
    bool nextIsIntensity(false);
    int echo_buffer[3] = {-1, -1, -1};
    int echo_buf_ind(0);
    bool done(false);
    while(!done)
    {
        // Read a line of data
        done = read_data_block(buffer, numBytesInLine);
        // Check if we've reached the end of the data
        if(numBytesInLine == 0)
        {
            err_output_ << "numBytesInLine is zero!\n";
        }
        // Process triplets of bytes until we encounter or overrun the end of
        // the line
        for (int ii = 0; ii < numBytesInLine;)
        {
            if(buffer[ii] == '\n' || buffer[ii + 1] == '\n')
            {
                // Line feed in the middle of a line? Why?
                throw MisplacedLineFeedError();
            }
            if(split_count == 0)
            {
                // Start of a value. Decide where to store the next value, and
                // if the previous is complete.
                if(buffer[ii] == '&')
                {
                    // Next echo
                    echo_buf_ind++;
                    ii++;
                }
                else if(echo_buffer[0] != -1)
                {
                    // Not the first value, so deal with the previous
                    if(nextIsIntensity)
                    {
                        data.write_intensity(current_intensity,
                                process_echo_buffer(echo_buffer,
                                    echo_buf_ind + 1));
                    }
                    else
                    {
                        data.write_range(current_range,
                                process_echo_buffer(echo_buffer,
                                    echo_buf_ind + 1));
                    }
                    echo_buf_ind = 0;
                    echo_buffer[0] = -1;
                    if(data.ranges_)
                    {
                        if(data.ranges_[current_range] > max_range_ &&
                                !nextIsIntensity)
                        {
                            err_output_ << "WARNING: Sensor::" << __func__ <<
                                "() Value at step " << current_range <<
                                " beyond maximum range: " <<
                                data.ranges_[current_range] << " (raw bytes: ";
                            if(split_count != 0)
                                err_output_ << split_value[0] << split_value[1] <<
                                    split_value[2] << ")\n";
                            else
                                err_output_ << buffer[0] << buffer[1] << buffer[2] <<
                                    ")\n";
                        }
                    }
                    if(nextIsIntensity)
                        current_intensity++;
                    else
                        current_range++;
                    // Alternate between range and intensity values
                    nextIsIntensity = !nextIsIntensity;
                }
            }
            if(ii == numBytesInLine - 2)       // Short 1 byte
            {
                split_value[0] = buffer[ii];
                split_value[1] = buffer[ii + 1];
                // Will be reset on the next iteration, after it's used
                split_count = 1;
                ii += 2;
            }
            else if(ii == numBytesInLine - 1)  // Short 2 bytes
            {
                split_value[0] = buffer[ii];
                // Will be reset on the next iteration, after it's used
                split_count = 2;
                ii += 1;
            }
            else
            {
                if(split_count == 1)
                {
                    split_value[2] = buffer[ii++];
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(split_value);
                }
                else if(split_count == 2)
                {
                    split_value[1] = buffer[ii++];
                    split_value[2] = buffer[ii++];
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(split_value);
                }
                else
                {
                    echo_buffer[echo_buf_ind] =
                        decode_3_byte_value(&buffer[ii]);
                    ii += 3;
                }
                // Reset this here now that it's been used
                split_count = 0;
            }
        }
        // End of this line. Go around again.
    }
    // The last piece should always be an intensity value (if it isn't
    // then the data count won't add up and an error will be thrown
    // below anyway).
    if(echo_buffer[0] != -1)
    {
        assert(nextIsIntensity == true);
        data.write_intensity(current_intensity,
            process_echo_buffer(echo_buffer, echo_buf_ind + 1));
        current_intensity++;
    }

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ << "() Read " <<
            current_range << " ranges and " << current_intensity <<
            " intensities (expected " << num_steps << ").\n";
    }
    if(current_range != num_steps || current_intensity != num_steps)
        throw DataCountError();
}


int Sensor::confirm_checksum(char const* buffer, int length,
        int expected_sum)
{
    int checksum = 0;
    // Start by adding the byte values
    for (int ii = 0; ii < length; ii++)
        checksum += buffer[ii];
    // Take the lowest 6 bits
    checksum &= 0x3F;
    // Add 0x30
    checksum += 0x30;

    if(verbose_)
    {
        err_output_ << "Sensor::" << __func__ <<
            "() Calculated checksum = " << checksum << " (" <<
            static_cast<char>(checksum) << "), given checksum = " <<
            expected_sum << " (" << static_cast<char> (expected_sum) <<
            ")\n";
    }
    if(checksum != expected_sum)
        throw ChecksumError(expected_sum, checksum);

    return checksum;
}

}; // namespace hokuyo_aist

