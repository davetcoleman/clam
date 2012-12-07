#include "nmeamessages.h"
#include <iostream>
#include "nmeasentence.h"
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <gbxutilacfr/exceptions.h>
#include <gbxutilacfr/mathdefs.h>

using namespace std;

namespace gbxgarminacfr {

// Get the useful bits from a GGA message
GenericData* extractGgaData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec )
{
    std::auto_ptr<GgaData> data( new GgaData );

    data->timeStampSec = timeSec;
    data->timeStampUsec = timeUsec;

    //Names for the tokens in the GGA message
    enum GgaTokens{MsgType=0,UTC,Lat,LatDir,Lon,LonDir,FixType,
                   NSatsUsed,HDOP,Hgt,M1,GeoidHgt,M2,DiffAge,DiffId};

    //UTC time 
    sscanf(sentence.getDataToken(UTC).c_str(),"%02d%02d%lf",
           &data->utcTimeHrs, &data->utcTimeMin, &data->utcTimeSec );

    //number of satellites in use
    data->satellites = atoi(sentence.getDataToken(NSatsUsed).c_str());

    // fix type
    switch ( sentence.getDataToken(FixType)[0] )
    {
    case '0': 
        data->fixType = Invalid;
        // NOTE: not processing the rest!
        data->latitude = 0.0;
        data->longitude = 0.0;
        data->altitude = 0.0;
        data->geoidalSeparation = 0.0;
        return data.release();
    case '1': 
        data->fixType = Autonomous;
        break;
    case '2': 
        data->fixType = Differential;
        break;
    default :
        throw gbxutilacfr::Exception( ERROR_INFO, "GGA sentence contains unknown GPS fix type: '"+sentence.getDataToken(FixType)+"'" );
    }
        
    //position
    int deg;
    double min;
    double dir;
    
    //latitude
    sscanf(sentence.getDataToken(Lat).c_str(),"%02d%lf",&deg,&min);
    dir = (*sentence.getDataToken(LatDir).c_str()=='N') ? 1.0 : -1.0;
    data->latitude=dir*(deg+(min/60.0));
    //longitude
    sscanf(sentence.getDataToken(Lon).c_str(),"%03d%lf",&deg,&min);
    dir = (*sentence.getDataToken(LonDir).c_str()=='E') ? 1.0 : -1.0;
    data->longitude=dir*(deg+(min/60.0));
    
    //altitude
    data->isAltitudeKnown = !sentence.isDataTokenEmpty(Hgt);
    if ( data->isAltitudeKnown )
        data->altitude=atof(sentence.getDataToken(Hgt).c_str());
    
    //geoidal Separation
    data->geoidalSeparation=atof(sentence.getDataToken(GeoidHgt).c_str());

    return data.release();
}

// VTG provides velocity and heading information
GenericData* extractVtgData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec )
{
    std::auto_ptr<VtgData> data( new VtgData );

    data->timeStampSec = timeSec;
    data->timeStampUsec = timeUsec;

    //Names for the VTG message items
    enum VtgTokens{MsgType=0,HeadingTrue,T,HeadingMag,M,SpeedKnots,
                   N,SpeedKPH,K,ModeInd};

    //Check for an empty string. Means that we can't tell anything useful.
    if ( sentence.isDataTokenEmpty(HeadingTrue) )
    {
        data->isValid = false;
        data->headingTrue = 0.0;
        data->headingMagnetic = 0.0;
        data->speed = 0.0;
        return data.release();
    }
    data->isValid = true;

    // true heading
    double headingRad = DEG2RAD(atof(sentence.getDataToken(HeadingTrue).c_str()));
    NORMALISE_ANGLE( headingRad );
    data->headingTrue=headingRad;

    // magnetic heading
    headingRad = DEG2RAD(atof(sentence.getDataToken(HeadingMag).c_str()));
    NORMALISE_ANGLE( headingRad );
    data->headingMagnetic=headingRad;

    //speed - converted to m/s
    data->speed=atof(sentence.getDataToken(SpeedKPH).c_str());
    data->speed*=(1000/3600.0);

    return data.release();
}

// RME message. This one is garmin specific... Give position error estimates
// See doc.dox for a discussion of the position errors as reported here. 
// Essentially the EPE reported by the garmin is a 1 sigma error (RMS) or a
// 68% confidence bounds.
GenericData* extractRmeData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec )
{
    std::auto_ptr<RmeData> data( new RmeData );

    data->timeStampSec = timeSec;
    data->timeStampUsec = timeUsec;

    //Names for the RME message items
    enum RmeTokens{MsgType=0,HError,M1,VError,M2,EPE,M3};
    
    if ( sentence.isDataTokenEmpty(HError) )
    {
        // No valid information
        data->isValid                      = false;
        data->isVerticalPositionErrorValid = false;
        data->horizontalPositionError      = 0.0;
        data->verticalPositionError        = 0.0;
        data->estimatedPositionError       = 0.0;
        return data.release();
    }
    data->isValid = true;

    data->horizontalPositionError = atof(sentence.getDataToken(HError).c_str());

    if ( sentence.isDataTokenEmpty(VError) )
    {
        data->isVerticalPositionErrorValid = false;
        data->verticalPositionError = -1;
    }
    else
    {
        data->isVerticalPositionErrorValid = true;
        data->verticalPositionError = atof(sentence.getDataToken(VError).c_str());
    }

    data->estimatedPositionError = atof(sentence.getDataToken(EPE).c_str());
    return data.release();
}

// RMC provides a combination of GGA and VTG data
GenericData* extractRmcData( const gbxgpsutilacfr::NmeaSentence& sentence, int timeSec, int timeUsec )
{
    std::auto_ptr<RmcData> data( new RmcData );

    data->timeStampSec = timeSec;
    data->timeStampUsec = timeUsec;

    //Names for the RMC message items
    enum RmcTokens{MsgType=0,UTC,Status,Lat,LatDir,Lon,LonDir,SpeedKnots,
                    HeadingTrue,DiffAge,MagneticVar,MagneticDir,ModeInd};

    //Check for an empty string. Means that we can't tell anything useful.
    if ( sentence.isDataTokenEmpty(HeadingTrue) )
    {
        data->isValid = false;
        data->headingTrue = 0.0;
        data->headingMagnetic = 0.0;
        data->speed = 0.0;
        return data.release();
    }
    data->isValid = true;

    //UTC time 
    sscanf(sentence.getDataToken(UTC).c_str(),"%02d%02d%lf",
           &data->utcTimeHrs, &data->utcTimeMin, &data->utcTimeSec );

    //position
    int deg;
    double min;
    double dir;
    
    //latitude
    sscanf(sentence.getDataToken(Lat).c_str(),"%02d%lf",&deg,&min);
    dir = (*sentence.getDataToken(LatDir).c_str()=='N') ? 1.0 : -1.0;
    data->latitude=dir*(deg+(min/60.0));
    //longitude
    sscanf(sentence.getDataToken(Lon).c_str(),"%03d%lf",&deg,&min);
    dir = (*sentence.getDataToken(LonDir).c_str()=='E') ? 1.0 : -1.0;
    data->longitude=dir*(deg+(min/60.0));

    // true heading
    double headingRad = DEG2RAD(atof(sentence.getDataToken(HeadingTrue).c_str()));
    NORMALISE_ANGLE( headingRad );
    data->headingTrue=headingRad;

    // magnetic heading
    double headingVar;
    try {
        // Magnetic deviation from true
        headingVar = DEG2RAD(atof(sentence.getDataToken(MagneticVar).c_str()));
        // Direction of magnetic deviation
        if (*sentence.getDataToken(MagneticDir).c_str() == 'E')
            headingRad -= headingVar;
        else 
            headingRad += headingVar;
    }
    catch ( const gbxgpsutilacfr::NmeaException& e ) {
        // If this occurs, cannot get magnetic heading. Set it to 0.
        headingRad = 0.0;
    }
    NORMALISE_ANGLE( headingRad );
    data->headingMagnetic=headingRad;

    //speed - converted from knots to m/s
    data->speed=atof(sentence.getDataToken(SpeedKnots).c_str());
    // knots to kph
    data->speed*=1.852;
    // kph to m/s
    data->speed*=(1000/3600.0);

    return data.release();
}

}

