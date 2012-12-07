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

#ifndef SENSOR_INFO_H__
#define SENSOR_INFO_H__

#if defined(WIN32)
    typedef unsigned char           uint8_t;
    typedef unsigned int            uint32_t;
    #if defined(HOKUYO_AIST_STATIC)
        #define HOKUYO_AIST_EXPORT
    #elif defined(HOKUYO_AIST_EXPORTS)
        #define HOKUYO_AIST_EXPORT       __declspec(dllexport)
    #else
        #define HOKUYO_AIST_EXPORT       __declspec(dllimport)
    #endif
#else
    #include <stdint.h>
    #define HOKUYO_AIST_EXPORT
#endif

#include <string>
#include <cstring>

/** @ingroup gbx_library_hokuyo_aist
@{
*/

namespace hokuyo_aist
{

/// Laser models
enum LaserModel
{
    MODEL_URG04LX, // Classic-URG
    MODEL_UBG04LXF01, // Rapid-URG
    MODEL_UHG08LX, // Hi-URG
    MODEL_UTM30LX, // Top-URG
    MODEL_UXM30LXE, // Tough-URG
    MODEL_UNKNOWN
};


HOKUYO_AIST_EXPORT inline char const* model_to_string(LaserModel model)
{
    switch(model)
    {
        case MODEL_URG04LX:
            return "URG-04LX";
        case MODEL_UBG04LXF01:
            return "UBG-04LX-F01";
        case MODEL_UHG08LX:
            return "UHG-08LX";
        case MODEL_UTM30LX:
            return "UTM-30LX";
        case MODEL_UXM30LXE:
            return "UXM-30LX-E";
        default:
            return "Unknown model";
    }
}


HOKUYO_AIST_EXPORT inline LaserModel string_to_model(char const* model)
{
    if(strncmp(model, "URG-04LX", 8) == 0)
        return MODEL_URG04LX;
    else if(strncmp(model, "UBG-04LX-F01", 8) == 0)
        return MODEL_UBG04LXF01;
    else if(strncmp(model, "UHG-08LX", 8) == 0)
        return MODEL_UHG08LX;
    else if(strncmp(model, "UTM-30LX", 8) == 0)
        return MODEL_UTM30LX;
    else if(strncmp(model, "UXM-30LX-E", 8) == 0)
        return MODEL_UXM30LXE;
    else
        return MODEL_UNKNOWN;
}


/// Sensor direction of rotation
enum RotationDirection
{
    CLOCKWISE,
    COUNTERCLOCKWISE
};


HOKUYO_AIST_EXPORT inline char const* rot_dir_to_string(RotationDirection dir)
{
    switch(dir)
    {
        case CLOCKWISE:
            return "Clockwise";
        case COUNTERCLOCKWISE:
            return "Counter-clockwise";
        default:
            return "Unknown";
    }
}


// Forward declaration
class Sensor;


/** @brief Sensor information.

Returned from a call to @get_sensor_info. Contains various information about the
laser scanner such as firmware version and maximum possible range. */
class HOKUYO_AIST_EXPORT SensorInfo
{
    public:
        friend class Sensor;

        SensorInfo();
        SensorInfo(SensorInfo const& rhs);

        /// @brief Assignment operator.
        SensorInfo& operator=(SensorInfo const& rhs);

        /// @brief Format the entire object into a string.
        std::string as_string();

        // Version details.
        /// Vendor name.
        std::string vendor;
        /// Product name.
        std::string product;
        /// Firmware version.
        std::string firmware;
        /// Protocol version in use.
        std::string protocol;
        /// Serial number of this device.
        std::string serial;

        // Specification details.
        /// Sensor model number.
        std::string model;
        /// Minimum detectable range (mm).
        unsigned int min_range;
        /// Maximum detectable range (mm).
        unsigned int max_range;
        /// Number of steps in a 360-degree scan.
        unsigned int steps;
        /// First scanable step of a full scan.
        unsigned int first_step;
        /// Last scanable step of a full scan.
        unsigned int last_step;
        /// Step number that points forward (typically the centre of a full
        /// scan).
        unsigned int front_step;
        /// Standard motor speed (rpm).
        unsigned int standard_speed;
        /// Rotation direction.
        RotationDirection rot_dir;

        // Status details.
        /// Operational status - illuminated or not.
        bool power;
        /// Current motor speed (rpm).
        unsigned int speed;
        /// Speed level (0 for default)
        unsigned short speed_level;
        /// Measurement state.
        std::string measure_state;
        /// Baud rate.
        unsigned int baud;
        /// Current sensor time (s).
        unsigned int time;
        /// Diagnostic status string.
        std::string sensor_diagnostic;

        // Calculated details
        /// Minimum possible scan angle (radians). Scans go anti-clockwise with
        /// negative angles on the right.
        double min_angle;
        /// Maximum possible scan angle (radians). Scans go anti-clockwise with
        /// negative angles on the right.
        double max_angle;
        /// Angle between two scan points (radians).
        double resolution;
        /// Time between two scan points (milliseconds).
        double time_resolution;
        /// Total number of steps in a full scan (lastStep - firstStep).
        unsigned int scanable_steps;
        /// Absolute maximum commandable step.
        unsigned int max_step;
        /// Detected model of the laser.
        LaserModel detected_model;

    private:
        void set_defaults();
        void calculate_values();
}; // class SensorInfo

}; // namespace hokuyo_aist

/** @} */

#endif // SENSOR_INFO_H__

