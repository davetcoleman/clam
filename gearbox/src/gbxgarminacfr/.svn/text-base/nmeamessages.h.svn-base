/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010  Alex Brooks, Alexei Makarenko, Tobias Kaupp, Duncan Mercer
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBXGPSUTILACFR_NMEAMESSAGES_H
#define GBXGPSUTILACFR_NMEAMESSAGES_H

#include <string>
#include <gbxgarminacfr/nmeasentence.h>

namespace gbxgarminacfr {

//! Possible types GenericData can contain
enum DataType 
{
    //! Contents of PGGGA message.
    GpGga,
    //! Contents of PGVTG message.
    GpVtg,
    //! Contents of PGRME message.
    PgRme,
    //! Contents of GPRMC message.
    GpRmc
};

//! Generic data type returned by a read
class GenericData
{
public:
    virtual ~GenericData() {};
    //! Returns data type.
    virtual DataType type() const=0;

private:
};

//! GPS fix types.
enum FixType 
{
    //! Invalid or not available
    Invalid,
    //! Autonomous position
    //! (This is the normal case for non-differential GPS)
    Autonomous,
    //! Differentially corrected
    Differential
};
std::string toString( const FixType &f );

//! Fix data structure. Note that when fixType is Invalid, all other data except the time stamps
//! are meaningless.
struct GgaData : public GenericData
{
public:
    DataType type() const { return GpGga; }

    //! Time (according to the computer clock) when data was measured.
    //! Number of seconds
    int timeStampSec;
    //! Time (according to the computer clock) when data was measured.
    //! Number of microseconds
    int timeStampUsec;

    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Hour [0..23]
    int utcTimeHrs;
    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Minutes [0..59]
    int utcTimeMin;
    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Seconds [0.0..59.9999(9)]
    double utcTimeSec;

    //! Latitude [degrees]
    double latitude;
    //! Longitude [degrees]
    double longitude;
    //! Altitude is meaningful if and only if isAltitudeKnown
    bool isAltitudeKnown;
    //! Altitude [metres above ellipsoid] (only meaningful if isAltitudeKnown)
    double altitude;
    
    //! Fix type. When fixType is Invalid, all other data except the time stamps
    //! are meaningless.
    FixType fixType;

    //! Number of satellites
    int satellites;

    //! Horizontal dilution of position [metres]
    double horizontalDilutionOfPosition;
    
    //! Height of geoid (mean sea level) above WGS84 ellipsoid [metres]
    double geoidalSeparation;    
};
std::string toString( const GgaData &d );
inline std::ostream &operator<<( std::ostream &s, const GgaData &d )
{ return s << toString(d); }
GenericData* extractGgaData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec );

//! Vector track and speed over ground data structure.
class VtgData : public GenericData
{
public:
    DataType type() const { return GpVtg; }

    //! Time (according to the computer clock) when data was measured.
    //! Number of seconds
    int timeStampSec;
    //! Time (according to the computer clock) when data was measured.
    //! Number of microseconds
    int timeStampUsec;

    //! When false, means that the GPS unit can't make a valid measurement
    //! (so all data other than the timestamp is meaningless).
    bool isValid;
    
    //! Heading/track/course with respect to true North [rad]
    double headingTrue; 
    //! Heading/track/course with respect to magnetic North [rad]
    double headingMagnetic; 
    //! Horizontal velocity [metres/second]
    double speed;
};
std::string toString( const VtgData &d );
inline std::ostream &operator<<( std::ostream &s, const VtgData &d )
{ return s << toString(d); }
GenericData* extractVtgData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec );

//! Gps data structure
//! (This one is Garmin-specific)
class RmeData : public GenericData
{
public:
    DataType type() const { return PgRme; }

    //! Time (according to the computer clock) when data was measured.
    //! Number of seconds
    int timeStampSec;
    //! Time (according to the computer clock) when data was measured.
    //! Number of microseconds
    int timeStampUsec;
    
    //! When false, means that the GPS unit can't make a valid measurement
    //! (so all data other than the timestamp is meaningless).
    bool isValid;

    //! When false, means that the GPS unit can't tell us anything
    //! about our vertical error
    bool isVerticalPositionErrorValid;
    
    //! Horizontal position error: one standard deviation [metres)]
    double horizontalPositionError;
    //! Vertical position error: one standard deviation [metres]
    double verticalPositionError;

    //! Estimated position error.
    double estimatedPositionError;
};
std::string toString( const RmeData &d );
inline std::ostream &operator<<( std::ostream &s, const RmeData &d )
{ return s << toString(d); }
GenericData* extractRmeData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec );

//! Gps data structure
class RmcData : public GenericData
{
public:
    DataType type() const { return GpRmc; }

    //! Time (according to the computer clock) when data was measured.
    //! Number of seconds
    int timeStampSec;
    //! Time (according to the computer clock) when data was measured.
    //! Number of microseconds
    int timeStampUsec;

    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Hour [0..23]
    int utcTimeHrs;
    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Minutes [0..59]
    int utcTimeMin;
    //! UTC time (according to the GPS device), reference is Greenwich.
    //! Seconds [0.0..59.9999(9)]
    double utcTimeSec;

    //! Latitude [degrees]
    double latitude;
    //! Longitude [degrees]
    double longitude;

    //! When false, means that the GPS unit can't make a valid measurement
    //! (so all data other than the timestamp is meaningless).
    bool isValid;
    
    //! Heading/track/course with respect to true North [rad]
    double headingTrue; 
    //! Heading/track/course with respect to magnetic North [rad]
    double headingMagnetic; 
    //! Horizontal velocity [metres/second]
    double speed;
};
std::string toString( const RmcData &d );
inline std::ostream &operator<<( std::ostream &s, const RmcData &d )
{ return s << toString(d); }
GenericData* extractRmcData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec );

}

#endif
