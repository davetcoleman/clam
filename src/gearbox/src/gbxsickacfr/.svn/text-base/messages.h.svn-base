/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef SICK_ACFR_DRIVER_MESSAGES_H
#define SICK_ACFR_DRIVER_MESSAGES_H

#include <string>
#include <vector>
#include <gbxsickacfr/sickdefines.h>
#include <gbxsickacfr/gbxserialdeviceacfr/gbxserialdeviceacfr.h>

namespace gbxsickacfr {

    std::string toHexString( const uChar *buf, int bufLen );
    inline std::string toHexString( const std::vector<uChar> &buf )
    {return toHexString( &(buf[0]), buf.size() );}

    //////////////////////////////////////////////////////////////////////
    // Generic LMS RxMsg classes:
    // 
    // - The SICK replies to commands with rxMsgs.
    // - RxMsgs are of a standard format, but some rxMsgs contain
    //   extra rxMsg-type-specific data.
    // - This is handled with the abstract class 'LmsRxMsgData':
    //   - The LmsRxMsg has a pointer to LmsRxMsgData, which should
    //     be cast to the appropriate type depending on the rxMsg type.
    //
    // (Note that in continuous mode, the measurements continuously
    //  sent out by the SICK are 'rxMsgs', even though there's no command).
    // 
    //////////////////////////////////////////////////////////////////////

    // Abstract class for representing rxMsg-type-specific data.
    class LmsRxMsgData : public IceUtil::Shared {
    public:
        virtual ~LmsRxMsgData() {}

        // human-readable string
        virtual std::string toString() const=0;

        virtual bool isError() const { return false; }
        virtual bool isWarn() const { return false; }

        // Returns a freshly allocated object of the same type
        //virtual LmsRxMsgData *clone() const=0;
    };
    typedef IceUtil::Handle<LmsRxMsgData> LmsRxMsgDataPtr;

    // This class represents rxMsgs which the SICK uses to reply to commands.
    // All rxMsg types have the information in this class.
    // Some rxMsgs also have extra data, which is stored in the 'data' member.
    class LmsRxMsg : public gbxserialdeviceacfr::RxMsg {
    public:
        LmsRxMsg()
            : status(0),
              data(NULL)
            {}
        // ~LmsRxMsg()
        //     { if (data) delete data; }

    // private:
    //     LmsRxMsg( const LmsRxMsg &other );
    //     LmsRxMsg &operator=( const LmsRxMsg &other );
    // public:

        uChar type;
        uChar status;
        LmsRxMsgDataPtr data;

        bool isError() const;
        bool isWarn() const;

        std::string toString() const;
    };
    std::string toString( const LmsRxMsg &r );
    typedef IceUtil::Handle<LmsRxMsg> LmsRxMsgPtr;
    inline std::string toString( const LmsRxMsgPtr &r )
    { return toString(*r); }

    //////////////////////////////////////////////////////////////////////
    // RxMsg-specific data classes
    //
    // - The set of classes below all inherit from the abstract
    //   'LmsRxMsgData' class.  They represent the data contained
    //   in specific rxMsg types.
    // - See 'parseRxMsg' in the .cpp file for details of which 
    //   classes go with which rxMsg codes.
    // 
    //////////////////////////////////////////////////////////////////////

    class LmsInitRxMsgData : public LmsRxMsgData {
    public:
        std::string description;

        std::string toString() const { return description; }
        LmsRxMsgData *clone() const { return new LmsInitRxMsgData(*this); }        
    };

    class LmsStatusRxMsgData : public LmsRxMsgData {
    public:
        std::string version;
        uChar       operatingMode;
        uChar       status;
        std::string manufacturer;
        uChar       variantType;
        uint16_t    pollution[POLLUTION_LENGTH];
        uint16_t    refPollution[REF_POLLUTION_LENGTH];
        uint16_t    calibPollution[CALIB_POLLUTION_LENGTH];
        uint16_t    calibRefPollution[CALIB_REF_POLLUTION_LENGTH];
        uint16_t    numMotorRevolutions;
        uint16_t    refScale1Dark100Pct;
        uint16_t    refScale2Dark100Pct;
        uint16_t    refScale1Dark66Pct;
        uint16_t    refScale2Dark66Pct;
        uint16_t    signalAmplitudePct;
        uint16_t    currentAngle;
        uint16_t    peakThreshold;
        uint16_t    angleOfMeasurement;
        uint16_t    calibSignalAmplitude;
        uint16_t    targetStopThreshold;
        uint16_t    targetPeakThreshold;
        uint16_t    actualStopThreshold;
        uint16_t    actualPeakThreshold;
        uChar       measuringMode;
        uint16_t    refSingleMeasuredValues;
        uint16_t    refMeanMeasuredValues;
        uint16_t    scanningAngle;
        uint16_t    angularResolution;
        uChar       restartMode;
        uChar       restartTime;
        uint16_t    baudRate;
        uChar       evaluationNumber;
        uChar       permanentBaudRate;
        uChar       lmsAddress;
        uChar       fieldSetNumber;
        uChar       currentMeasuredValueUnit;
        uChar       laserSwitchOff;
        std::string softwareVersion;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsStatusRxMsgData(*this); }
    };

    class LmsSwitchOperatingModeRxMsgData : public LmsRxMsgData {
    public:
        uChar success;

        bool isError() const { return success != OPERATING_MODE_RESPONSE_SUCCESS; }
        std::string toString() const { return modeSwitchSuccessToString(success); }
        LmsRxMsgData *clone() const { return new LmsSwitchOperatingModeRxMsgData(*this); }
    };

    class LmsConfigurationData : public LmsRxMsgData {
    public:

        // Default values for all these fuckers
        LmsConfigurationData();
        bool operator==( const LmsConfigurationData &o ) const;
        bool operator!=( const LmsConfigurationData &o ) const
            { return !(operator==(o)); }

        uint16_t blanking;
        uChar    sensitivity;
        uChar    availability;
        uChar    measuringMode;
        uChar    measuredValueUnit;
        uChar    transientFieldSet;
        uChar    subtractiveFields;   // 14
        uChar    multipleEvaluation;
        uChar    restart;
        uChar    restartTime;
        uChar    multipleEvaluationForSuppressed;
        uChar    contourARef;
        uChar    contourAPosToleranceBand; // 20
        uChar    contourANegToleranceBand;
        uChar    contourAStartAngle;
        uChar    contourAStopAngle;
        uChar    contourBRef;
        uChar    contourBPosToleranceBand;
        uChar    contourBNegToleranceBand;
        uChar    contourBStartAngle;
        uChar    contourBStopAngle;
        uChar    contourARef2;
        uChar    contourAPosToleranceBand2; // 30
        uChar    contourCNegToleranceBand;
        uChar    contourCStartAngle;
        uChar    contourCStopAngle;
        uChar    pixelOrientedEvaluation;
        uChar    singleMeasuredValueEvaluation;
        uint16_t restartTimeFields;
        uint16_t multipleEvaluationForDazzle;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsConfigurationData(*this); }
    };

    class LmsConfigurationRxMsgData : public LmsRxMsgData {
    public:
        LmsConfigurationData config;
        uChar                configSuccess;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsConfigurationRxMsgData(*this); }        
        bool isError() const { return configSuccess != CONFIGURATION_SUCCESS; }
    };

    class LmsSwitchVariantRxMsgData : public LmsRxMsgData {
    public:
        uChar            success;
        uint16_t         scanningAngle;
        uint16_t         angularResolution;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsSwitchVariantRxMsgData(*this); }        
        bool isError() const { return success != SWITCH_VARIANT_SUCCESS; }
    };

    class LmsMeasurementData : public LmsRxMsgData {
    public:
        
        // ranges in metres
        std::vector<float> ranges;
        std::vector<uChar> intensities;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsMeasurementData(*this); }        
    };

    class LmsErrorRxMsgData : public LmsRxMsgData {
    public:
        
        std::vector<uChar> errorTypes;
        std::vector<uChar> errorCodes;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsErrorRxMsgData(*this); }        
        bool isError() const;
        bool isWarn() const;
    };

    class LmsOperatingDataCounterData : public LmsRxMsgData {
    public:
        
        int hoursOfOperation;
        int numSwitchOns;

        std::string toString() const;
        LmsRxMsgData *clone() const { return new LmsOperatingDataCounterData(*this); }        
    };

    ////////////////////////////////////////////////////////////////////////////////

    // If a complete telegram was found, returns it
    // (also sets bytesParsed regardless of whether a complete message was found)
    LmsRxMsgPtr parseBufferForRxMsgs( const uChar  *buffer,
                                      int           bufferLength,
                                      int          &bytesParsed );

    void constructTelegram( std::vector<uChar>       &buffer,
                            const std::vector<uChar> &commandAndData );

    // SICK parameters can be changed in installation mode.
    void constructRequestInstallationMode( std::vector<uChar> &commandAndData );

    void constructRequestContinuousMode( std::vector<uChar> &commandAndData );
    void constructRequestMeasuredOnRequestMode( std::vector<uChar> &commandAndData );

    void constructInitAndReset( std::vector<uChar> &commandAndData );

    void constructStatusRequest( std::vector<uChar> &commandAndData );

    void constructConfigurationRequest( std::vector<uChar> &commandAndData );

    void constructConfigurationCommand( const LmsConfigurationData &c,
                                        std::vector<uChar> &commandAndData );

    void constructRequestErrorMessage( std::vector<uChar> &commandAndData );

    void constructSwitchVariant( uint16_t scanningAngle,
                                 uint16_t angularResolution,
                                 std::vector<uChar> &commandAndData );

    void constructRequestOperatingDataCounter( std::vector<uChar> &commandAndData );
        
    void constructRequestBaudRate( std::vector<uChar> &commandAndData, int baudRate );
}

#endif
