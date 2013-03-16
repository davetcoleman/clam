/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include "messages.h"
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <IceUtil/Time.h>
#include <gbxutilacfr/exceptions.h>
#include <assert.h>

using namespace std;

namespace gbxsickacfr {

namespace {
    const int PREAMBLE_LENGTH=4;
    const int COMMAND_LENGTH=1;
    const int CHECKSUM_LENGTH=2;
    
    const int MIN_RXMSG_SIZE = PREAMBLE_LENGTH + COMMAND_LENGTH + CHECKSUM_LENGTH;
    
    // For computing checksums
    #define CRC16_GEN_POL     0x8005
    #define CRC16_GEN_POL0    0x80
    #define CRC16_GEN_POL1    0x05

    // Give it the buffer and the buffer length.
    // Returns the checksum.
    //
    // (Note: this function is copied directly from the SICK manual)
    //
    int computeSickChecksum(const unsigned char *CommData, int uLen)
    {
        unsigned char abData[2] = {0, 0}, uCrc16[2] = {0, 0};
  
        while(uLen--) {
            abData[0] = abData[1];
            abData[1] = *CommData++;
            if(uCrc16[0] & 0x80) {
                uCrc16[0] <<= 1;
                if(uCrc16[1] & 0x80)
                    uCrc16[0] |= 0x01;
                uCrc16[1] <<= 1;
                uCrc16[0] ^= CRC16_GEN_POL0;
                uCrc16[1] ^= CRC16_GEN_POL1;
            } 
            else {
                uCrc16[0] <<= 1;
                if(uCrc16[1] & 0x80)
                    uCrc16[0] |= 0x01;
                uCrc16[1] <<= 1;
            }
            uCrc16[0] ^= abData[0];
            uCrc16[1] ^= abData[1];
        }
        return (((int)uCrc16[0]) * 256 + ((int)uCrc16[1]));
    }

}

//////////////////////////////////////////////////////////////////////
// Printing Functions
//////////////////////////////////////////////////////////////////////

std::string toHexString( const uChar *buf, int bufLen )
{
    stringstream ss;
    ss << "[ ";
    for ( int i=0; i < bufLen; i++ )
    {
        ss <<hex<<std::setfill('0')<<std::setw(2)<<(int)(buf[i])<<" ";
    }
    ss << "]";
    return ss.str();
}

std::string toAsciiString( const uChar *buf, int bufLen )
{
    stringstream ss;
    ss << "[ ";
    for ( int i=0; i < bufLen; i++ )
    {
        ss <<buf[i]<<"  ";
    }
    ss << "]";
    return ss.str();
}

std::string
toHexString( const uChar &c )
{
    stringstream ss;
    ss << hex << ((int)c);
    return ss.str();
}

std::string
toString( const uint16_t *s, int len )
{
    stringstream ss;
    for ( int i=0; i < len; i++ )
    {
        ss << s[i] << " ";
    }
    return ss.str();
}

//////////////////////////////////////////////////////////////////////
// Buffer-Reading Functions
//////////////////////////////////////////////////////////////////////

uint16_t getWord( const uChar *buf, int &pos )
{
    uint16_t word = (uint16_t)( buf[pos] | (buf[pos+1]<<8) );
    pos += sizeof(uint16_t);
    return word;
}

//
// NOTE: grabs an extra byte coz the strings coming from the SICK are null-terminated.
//
void
getString( int len, const uChar *buf, int &pos, std::string &str )
{
    str.resize( len );
    for ( int i=0; i < len; i++ )
    {
        str[i] = buf[pos+i];
    }
    pos += len+1; // +1 for null-terminator
}

//////////////////////////////////////////////////////////////////////
// LmsRxMsg Stuff
//////////////////////////////////////////////////////////////////////

// LmsRxMsg &
// LmsRxMsg::operator=( const LmsRxMsg &other )
// {
//     type = other.type;
//     status = other.status;
// //    if ( data ) delete data;
//     if ( other.data == NULL )
//         data = other.data;
//     else
//         data = other.data->clone();
//     return *this;
// }
// LmsRxMsg::LmsRxMsg( const LmsRxMsg &other )
// {
//     *this = other;
// }

std::string
toString( const LmsRxMsg &r )
{
    stringstream ss;
    ss << endl;
    ss << "type: " << cmdToString(r.type) <<"(0x"<<hex<<(int)(r.type)<<")"<< endl;
    if ( r.type == ACK || r.type == NACK )
        return ss.str();

    ss << " status: 0x" <<hex<<(int)(r.status)<<dec<<": "<< statusToString(r.status) << endl
       << " data: ";
    if ( r.data )
        ss << r.data->toString();
    else
        ss << "  [none]";
    return ss.str();
}

std::string
LmsRxMsg::toString() const 
{
    return gbxsickacfr::toString(*this);
}

bool
LmsRxMsg::isError() const
{
    uChar generalStatus = (uChar)( status & STATUS_GENERAL_MASK );
    if ( generalStatusIsError( generalStatus ) )
    {
        return true;
    }
    if ( status & STATUS_POLLUTION_MASK )
    {
        return true;
    }
    if ( data &&
         data->isError() )
    {
        return true;
    }
    if ( type == RESP_INCORRECT_COMMAND )
    {
        return true;
    }

    return false;
}

bool
LmsRxMsg::isWarn() const
{
    uChar generalStatus = (uChar)( status & STATUS_GENERAL_MASK );
    if ( generalStatusIsWarn( generalStatus ) )
    {
        return true;
    }
    if ( data &&
         data->isWarn() )
    {
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////
// RxMsg Data Parsing Stuff
//////////////////////////////////////////////////////////////////////

std::string
LmsStatusRxMsgData::toString() const
{
    stringstream ss;
    ss << endl;
    ss << "  Version:                  " << version << endl
       << "  Operating Mode:           " << operatingModeToString(operatingMode) << endl
       << "  Status:                   " << statusToString(status) << endl
       << "  Manufacturer:             " << manufacturer << endl
       << "  VariantType:              " << toHexString(variantType) << endl
       << "  Pollution:                " << gbxsickacfr::toString(pollution,POLLUTION_LENGTH) << endl
       << "  RefPollution:             " << gbxsickacfr::toString(refPollution,REF_POLLUTION_LENGTH) << endl
       << "  CalibPollution:           " << gbxsickacfr::toString(calibPollution,CALIB_POLLUTION_LENGTH) << endl
       << "  CalibRefPollution:        " << gbxsickacfr::toString(calibRefPollution,CALIB_REF_POLLUTION_LENGTH) << endl
       << "  NumMotorRevolutions:      " << numMotorRevolutions << endl
       << "  RefScale1Dark100Pct:      " << refScale1Dark100Pct << endl
       << "  RefScale2Dark100Pct:      " << refScale2Dark100Pct << endl
       << "  RefScale1Dark66Pct:       " << refScale1Dark66Pct << endl
       << "  RefScale2Dark66Pct:       " << refScale2Dark66Pct << endl
       << "  SignalAmplitudePct:       " << signalAmplitudePct << endl
       << "  CurrentAngle:             " << currentAngle << endl
       << "  PeakThreshold:            " << peakThreshold << endl
       << "  AngleOfMeasurement:       " << angleOfMeasurement << endl
       << "  CalibSignalAmplitude:     " << calibSignalAmplitude << endl
       << "  TargetStopThreshold:      " << targetStopThreshold << endl
       << "  TargetPeakThreshold:      " << targetPeakThreshold << endl
       << "  ActualStopThreshold:      " << actualStopThreshold << endl
       << "  ActualPeakThreshold:      " << actualPeakThreshold << endl
       << "  MeasuringMode:            " << toHexString(measuringMode) << endl
       << "  RefSingleMeasuredValues:  " << refSingleMeasuredValues << endl
       << "  RefMeanMeasuredValues:    " << refMeanMeasuredValues << endl
       << "  ScanningAngle:            " << scanningAngle << endl
       << "  AngularResolution:        " << angularResolution << endl
       << "  RestartMode:              " << toHexString(restartMode) << endl
       << "  RestartTime:              " << toHexString(restartTime) << endl
       << "  BaudRate:                 " << baudRateToInt(baudRate) << endl
       << "  EvaluationNumber:         " << toHexString(evaluationNumber) << endl
       << "  PermanentBaudRate:        " << permanentBaudRateToString(permanentBaudRate) << endl
       << "  LmsAddress:               " << toHexString(lmsAddress) << endl
       << "  FieldSetNumber:           " << toHexString(fieldSetNumber) << endl
       << "  CurrentMeasuredValueUnit: " << toHexString(currentMeasuredValueUnit) << endl
       << "  LaserSwitchOff:           " << toHexString(laserSwitchOff) << endl
       << "  SoftwareVersion:          " << softwareVersion << endl;
        
    return ss.str();
}

LmsStatusRxMsgData *
parseStatusRxMsgData( const uChar *buf, int len )
{
//     cout<<"TRACE(messages.cpp): parseStatusRxMsgData():" << endl;
//     cout<<"  " << toHexString( buf, 20 ) << endl;
//     cout<<"  " << toAsciiString( buf, 20 ) << endl;

    LmsStatusRxMsgData *r = new LmsStatusRxMsgData;
    int pos = 0;

    getString( VERSION_LENGTH, buf, pos, r->version );
    r->operatingMode = buf[pos++];
    r->status = buf[pos++];
    getString( MANUFACTURER_LENGTH, buf, pos, r->manufacturer );
    r->variantType = buf[pos++];

    for ( int i=0; i < POLLUTION_LENGTH; i++ )
        r->pollution[i] = getWord(buf,pos);
    for ( int i=0; i < REF_POLLUTION_LENGTH; i++ )
        r->refPollution[i] = getWord(buf,pos);

    for ( int i=0; i < CALIB_POLLUTION_LENGTH; i++ )
        r->calibPollution[i] = getWord(buf,pos);
    for ( int i=0; i < CALIB_REF_POLLUTION_LENGTH; i++ )
        r->calibRefPollution[i] = getWord(buf,pos);

    r->numMotorRevolutions = getWord(buf,pos);
    pos+=sizeof(uint16_t);
    r->refScale1Dark100Pct = getWord(buf,pos);
    pos+=sizeof(uint16_t);
    r->refScale2Dark100Pct = getWord(buf,pos);
    r->refScale1Dark66Pct = getWord(buf,pos);
    pos+=sizeof(uint16_t);
    r->refScale2Dark66Pct = getWord(buf,pos);
    r->signalAmplitudePct = getWord(buf,pos);
    r->currentAngle = getWord(buf,pos);
    r->peakThreshold = getWord(buf,pos);
    r->angleOfMeasurement = getWord(buf,pos);
    r->calibSignalAmplitude = getWord(buf,pos);
    r->targetStopThreshold = getWord(buf,pos);
    r->targetPeakThreshold = getWord(buf,pos);
    r->actualStopThreshold = getWord(buf,pos);
    r->actualPeakThreshold = getWord(buf,pos);
    pos+=sizeof(uChar);
    r->measuringMode = buf[pos++];
    r->refSingleMeasuredValues = getWord(buf,pos);
    r->refMeanMeasuredValues = getWord(buf,pos);
    r->scanningAngle = getWord(buf,pos);
    r->angularResolution = getWord(buf,pos);
    r->restartMode = buf[pos++];
    r->restartTime = buf[pos++];
    pos+=2*sizeof(uChar);
    r->baudRate = getWord(buf,pos);
    r->evaluationNumber = buf[pos++];
    r->permanentBaudRate = buf[pos++];
    r->lmsAddress = buf[pos++];
    r->fieldSetNumber = buf[pos++];
    r->currentMeasuredValueUnit = buf[pos++];
    r->laserSwitchOff = buf[pos++];
    getString(SOFTWARE_VERSION_LENGTH,buf,pos,r->softwareVersion);
//    pos+=7*sizeof(uint16_t);
//    pos += 21; // Not sure exactly where these come from...
//    assert( pos == len );

    return r;
}

LmsConfigurationData::LmsConfigurationData()
    : blanking(0),
      sensitivity(0),
      availability(0),
      measuringMode(0),
      measuredValueUnit(0),
      transientFieldSet(0),
      subtractiveFields(0),   // 14
      multipleEvaluation(0x02),
      restart(0x02),
      restartTime(0),
      multipleEvaluationForSuppressed(0),
      contourARef(0),
      contourAPosToleranceBand(0), // 20
      contourANegToleranceBand(0),
      contourAStartAngle(0),
      contourAStopAngle(0),
      contourBRef(0),
      contourBPosToleranceBand(0),
      contourBNegToleranceBand(0),
      contourBStartAngle(0),
      contourBStopAngle(0),
      contourARef2(0),
      contourAPosToleranceBand2(0), // 30
      contourCNegToleranceBand(0),
      contourCStartAngle(0),
      contourCStopAngle(0),
      pixelOrientedEvaluation(0),
      singleMeasuredValueEvaluation(0),
      restartTimeFields(0),
      multipleEvaluationForDazzle(0x0002)
{}

std::string
LmsConfigurationData::toString() const
{
    stringstream ss;
    ss << endl;

    ss << "  blanking                        : " << blanking << endl
       << "  sensitivity                     : " << sensitivityToString(sensitivity) << endl
       << "  availability                    : " << toHexString(availability) << endl
       << "  measuringMode                   : " << measuringModeToString(measuringMode) << endl
       << "  measuredValueUnit               : " << measuredValueUnitToString(measuredValueUnit) << endl
       << "  transientFieldSet               : " << toHexString(transientFieldSet) << endl
       << "  subtractiveFields               : " << toHexString(subtractiveFields) << endl
       << "  multipleEvaluation              : " << toHexString(multipleEvaluation) << endl
       << "  restart                         : " << toHexString(restart) << endl
       << "  restartTime                     : " << toHexString(restartTime) << endl
       << "  multipleEvaluationForSuppressed : " << toHexString(multipleEvaluationForSuppressed) << endl
       << "  contourARef                     : " << toHexString(contourARef) << endl
       << "  contourAPosToleranceBand        : " << toHexString(contourAPosToleranceBand) << endl
       << "  contourANegToleranceBand        : " << toHexString(contourANegToleranceBand) << endl
       << "  contourAStartAngle              : " << toHexString(contourAStartAngle) << endl
       << "  contourAStopAngle               : " << toHexString(contourAStopAngle) << endl
       << "  contourBRef                     : " << toHexString(contourBRef) << endl
       << "  contourBPosToleranceBand        : " << toHexString(contourBPosToleranceBand) << endl
       << "  contourBNegToleranceBand        : " << toHexString(contourBNegToleranceBand) << endl
       << "  contourBStartAngle              : " << toHexString(contourBStartAngle) << endl
       << "  contourBStopAngle               : " << toHexString(contourBStopAngle) << endl
       << "  contourARef2                    : " << toHexString(contourARef2) << endl
       << "  contourAPosToleranceBand2       : " << toHexString(contourAPosToleranceBand2) << endl
       << "  contourCNegToleranceBand        : " << toHexString(contourCNegToleranceBand) << endl
       << "  contourCStartAngle              : " << toHexString(contourCStartAngle) << endl
       << "  contourCStopAngle               : " << toHexString(contourCStopAngle) << endl
       << "  pixelOrientedEvaluation         : " << toHexString(pixelOrientedEvaluation) << endl
       << "  singleMeasuredValueEvaluation   : " << toHexString(singleMeasuredValueEvaluation) << endl
       << "  restartTimeFields               : " << restartTimeFields << endl
       << "  multipleEvaluationForDazzle     : " << multipleEvaluationForDazzle << endl;
    return ss.str();
}

LmsRxMsgData *
parseLmsConfigurationData( const uChar *buf, int len )
{
    LmsConfigurationData *c = new LmsConfigurationData;
    int pos = 0;

    c->blanking = getWord(buf,pos);
    pos++;
    c->sensitivity = buf[pos++];
    c->availability = buf[pos++];
    c->measuringMode = buf[pos++];
    c->measuredValueUnit = buf[pos++];
    c->transientFieldSet = buf[pos++];
    c->subtractiveFields = buf[pos++];   // 14
    c->multipleEvaluation = buf[pos++];
    c->restart = buf[pos++];
    c->restartTime = buf[pos++];
    c->multipleEvaluationForSuppressed = buf[pos++];
    c->contourARef = buf[pos++];
    c->contourAPosToleranceBand = buf[pos++]; // 20
    c->contourANegToleranceBand = buf[pos++];
    c->contourAStartAngle = buf[pos++];
    c->contourAStopAngle = buf[pos++];
    c->contourBRef = buf[pos++];
    c->contourBPosToleranceBand = buf[pos++];
    c->contourBNegToleranceBand = buf[pos++];
    c->contourBStartAngle = buf[pos++];
    c->contourBStopAngle = buf[pos++];
    c->contourARef2 = buf[pos++];
    c->contourAPosToleranceBand2 = buf[pos++]; // 30
    c->contourCNegToleranceBand = buf[pos++];
    c->contourCStartAngle = buf[pos++];
    c->contourCStopAngle = buf[pos++];
    c->pixelOrientedEvaluation = buf[pos++];
    c->singleMeasuredValueEvaluation = buf[pos++];
    c->restartTimeFields = getWord(buf,pos);
    c->multipleEvaluationForDazzle = getWord(buf,pos);

    // cout<<"TRACE(messages.cpp): pos: " << pos << ", len: " << len << endl;
    assert( pos == len );

    return c;
}

bool
LmsConfigurationData::operator==( const LmsConfigurationData &o ) const
{
    return ( blanking == o.blanking &&
             sensitivity == o.sensitivity &&
             availability == o.availability &&
             measuringMode == o.measuringMode &&
             measuredValueUnit == o.measuredValueUnit &&
             transientFieldSet == o.transientFieldSet &&
             subtractiveFields == o.subtractiveFields &&
             multipleEvaluation == o.multipleEvaluation &&
             restart == o.restart &&
             restartTime == o.restartTime &&
             multipleEvaluationForSuppressed == o.multipleEvaluationForSuppressed &&
             contourARef == o.contourARef &&
             contourAPosToleranceBand == o.contourAPosToleranceBand &&
             contourANegToleranceBand == o.contourANegToleranceBand &&
             contourAStartAngle == o.contourAStartAngle &&
             contourAStopAngle == o.contourAStopAngle &&
             contourBRef == o.contourBRef &&
             contourBPosToleranceBand == o.contourBPosToleranceBand &&
             contourBNegToleranceBand == o.contourBNegToleranceBand &&
             contourBStartAngle == o.contourBStartAngle &&
             contourBStopAngle == o.contourBStopAngle &&
             contourARef2 == o.contourARef2 &&
             contourAPosToleranceBand2 == o.contourAPosToleranceBand2 &&
             contourCNegToleranceBand == o.contourCNegToleranceBand &&
             contourCStartAngle == o.contourCStartAngle &&
             contourCStopAngle == o.contourCStopAngle &&
             pixelOrientedEvaluation == o.pixelOrientedEvaluation &&
             singleMeasuredValueEvaluation == o.singleMeasuredValueEvaluation &&
             restartTimeFields == o.restartTimeFields &&
             multipleEvaluationForDazzle == o.multipleEvaluationForDazzle );
}

std::string
LmsConfigurationRxMsgData::toString() const
{
    stringstream ss;
    ss << endl;
    ss << "success: " << configurationSuccessToString(configSuccess) << endl;
    ss << "config: " << config.toString();
    return ss.str();
}

LmsRxMsgData *
parseLmsConfigurationRxMsgData( const uChar *buf, int len )
{
    LmsConfigurationRxMsgData *c = new LmsConfigurationRxMsgData;
    c->configSuccess = buf[0];

    LmsConfigurationData *config = (LmsConfigurationData*)parseLmsConfigurationData( &(buf[1]), len-1 );
    c->config = *config;
    delete config;

    return c;
}

LmsRxMsgData *
parseLmsSwitchVariantRxMsgData( const uChar *buf, int len )
{
    LmsSwitchVariantRxMsgData *c = new LmsSwitchVariantRxMsgData;
    int pos=0;

    c->success = buf[pos++];
    c->scanningAngle = getWord( buf, pos );
    c->angularResolution = getWord( buf, pos );

    return c;
}

std::string
LmsSwitchVariantRxMsgData::toString() const 
{
    stringstream ss;
    ss << endl;
    ss << "    success: " << switchVariantSuccessToString(success) << endl
       << "    scanningAngle: " << scanningAngle << "deg" << endl
       << "    angularResolution: " << angularResolutionToDoubleInDegrees(angularResolution) << "deg";

    return ss.str();
}

LmsRxMsgData *
parseLmsMeasurementData( const uChar *buf, int len )
{
    LmsMeasurementData *d = new LmsMeasurementData;
    
    uChar measurementMode = (uChar)( buf[1] >> 6 );
    double rangeConversion;
    if ( measurementMode == MEASURED_VALUE_UNIT_MM )
    {
        rangeConversion = 1.0/1000.0;
    }
    else if ( measurementMode == MEASURED_VALUE_UNIT_CM )
    {
        rangeConversion = 1.0/100.0;
    }
    else
    {
        stringstream ss;
        ss << "Unknown measurement mode: " << measurementMode;
        throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
    }

    bool scanType = (buf[1] >> 5) & 0x01;
    // uChar partialScanNumber = (buf[1] >> 3) & 0x03;
    if ( scanType != 0 )
    {
        stringstream ss;
        ss << "parseLmsMeasurementData(): Only know how to deal with complete scan.";
        throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
    }

    int numMeasurements = ((buf[1]&0x80)<<8) | (buf[0]);
    int pos = 2;
    
    d->ranges.resize( numMeasurements );
    d->intensities.resize( numMeasurements );

    for ( int i=0; i < numMeasurements; i++ )
    {
        uChar loByte = buf[pos];
        uChar hiByte = buf[pos+1];
        d->ranges[i] = (float)(( ((hiByte&0x1f)<<8) + loByte ) * rangeConversion);
        d->intensities[i] = (hiByte & 0xe0) >> 5;

        pos += sizeof(uint16_t);
    }

    assert( pos == len );

    return d;
}

std::string
LmsMeasurementData::toString() const
{
    return "LmsMeasurementData";
}

LmsRxMsgData *
parseLmsErrorRxMsgData( const uChar *buf, int len )
{
    LmsErrorRxMsgData *e = new LmsErrorRxMsgData;
    int pos = 0;

    e->errorTypes.resize( len/2 );
    e->errorCodes.resize( len/2 );
    for ( size_t i=0; i < e->errorTypes.size(); i++ )
    {
        e->errorTypes[i] = buf[pos++];
        e->errorCodes[i] = buf[pos++];
    }
    assert( pos == len );

    return e;
}

std::string
LmsErrorRxMsgData::toString() const
{
    stringstream ss;
    ss << "num errors: " << errorTypes.size() << endl;

    for ( size_t i=0; i < errorTypes.size(); i++ )
    {
        ss << "    " << i << ": " << errorTypeToString(errorTypes[i]) << ": " << errorCodeToString(errorCodes[i]) << endl;
    }
    return ss.str();
}

bool
LmsErrorRxMsgData::isWarn() const
{
    assert( errorTypes.size() == errorCodes.size() );

    for ( size_t i=0; i < errorTypes.size(); i++ )
    {
        // Check for still-relevant errors
        if ( ! (errorTypes[i] & ERROR_TYPE_NO_LONGER_RELEVANT_MASK) )
        {
            // Are they real errors or just warnings/info?
            if ( errorTypeIsWarn( errorTypes[i] ) )
                return true;
        }
    }
    return false;
}

bool
LmsErrorRxMsgData::isError() const
{
    assert( errorTypes.size() == errorCodes.size() );

    for ( size_t i=0; i < errorTypes.size(); i++ )
    {
        // Check for still-relevant errors
        if ( ! (errorTypes[i] & ERROR_TYPE_NO_LONGER_RELEVANT_MASK) )
        {
            // Are they real errors or just warnings/info?
            if ( errorTypeIsError( errorTypes[i] ) )
                return true;
        }
    }
    return false;
}

LmsRxMsgData *
parseLmsOperatingDataCounterData( const uChar *buf, int len )
{
    assert( len == 4 );
    LmsOperatingDataCounterData *d = new LmsOperatingDataCounterData;

    int pos = 0;
    d->hoursOfOperation = getWord( buf, pos ) * 2;
    d->numSwitchOns = getWord( buf, pos );

    assert( pos == len );
    
    return d;
}

std::string
LmsOperatingDataCounterData::toString() const
{
    stringstream ss;
    ss << "Hours of operation: " << hoursOfOperation << ", numSwitchOns: " << numSwitchOns;
    return ss.str();
}

//////////////////////////////////////////////////////////////////////

void
constructTelegram( std::vector<uChar>       &buffer,
                   const std::vector<uChar> &commandAndData )
{
    buffer.resize( PREAMBLE_LENGTH + 
                   commandAndData.size() + 
                   CHECKSUM_LENGTH );

    int pos=0;
    buffer[pos++] = STX;
    buffer[pos++] = ADDRESS;
    buffer[pos++] = (unsigned char)(commandAndData.size() & 0x00ff);
    buffer[pos++] = (unsigned char)(commandAndData.size() >> 8);

    memcpy( &(buffer[pos]),
            &(commandAndData[0]),
            commandAndData.size()*sizeof(uChar) );

    int checksum = computeSickChecksum( &(buffer[0]),
                                        buffer.size()-CHECKSUM_LENGTH );
    buffer[ buffer.size()-2 ] = (unsigned char)(checksum & 0xFF);
    buffer[ buffer.size()-1 ] = (unsigned char)(checksum >> 8);
}

void
parseRxMsg( uChar        rxMsgCode,
            const uChar *rxMsgData,
            int          rxMsgDataSize,
            LmsRxMsg    &rxMsg )
{
//     cout<<"TRACE(messages.cpp): parseRxMsg(code="<<hex<<(int)(rxMsgCode)<<")" << endl;
//     cout<<"TRACE(messages.cpp): rxMsgDataSize: " << dec << rxMsgDataSize << endl;

    switch( rxMsgCode )
    {
    case ACK_INIT_AND_RESET:
    {
        LmsInitRxMsgData *rData = new LmsInitRxMsgData;
        rData->description.resize( rxMsgDataSize );
        memcpy( &(rData->description[0]), rxMsgData, rxMsgDataSize*sizeof(uChar) );
        rxMsg.data = rData;
        break;
    }
    case RESP_SOFTWARE_RESET_CONFIRM:
    {
        break;
    }
    case ACK_REQUEST_LMS_STATUS:
    {
        rxMsg.data = parseStatusRxMsgData( rxMsgData, rxMsgDataSize );
        break;
    }
    case ACK_REQUEST_LMS_CONFIGURATION:
    {
        rxMsg.data = parseLmsConfigurationData( rxMsgData, rxMsgDataSize );
        break;
    }
    case ACK_CONFIGURE_LMS:
    {
        rxMsg.data = parseLmsConfigurationRxMsgData( rxMsgData, rxMsgDataSize );
        break;
    }
    case ACK_SWITCH_OPERATING_MODE:
    {
        LmsSwitchOperatingModeRxMsgData *rData = new LmsSwitchOperatingModeRxMsgData;
        rData->success = rxMsgData[0];
        rxMsg.data = rData;
        break;
    }
    case ACK_SWITCH_VARIANT:
    {
        rxMsg.data = parseLmsSwitchVariantRxMsgData( rxMsgData,
                                                           rxMsgDataSize );
        break;
    }
    case ACK_REQUEST_ERROR_OR_TEST_MESSAGE:
    {
        rxMsg.data = parseLmsErrorRxMsgData( rxMsgData, rxMsgDataSize );
        break;
    }
    case ACK_REQUEST_MEASURED_VALUES:
    {
        // These get pumped out in continuous mode
        rxMsg.data = parseLmsMeasurementData( rxMsgData,
                                                 rxMsgDataSize );
        break;
    }
    case ACK_REQUEST_OPERATING_DATA_COUNTER:
    {
        rxMsg.data = parseLmsOperatingDataCounterData( rxMsgData,
                                                          rxMsgDataSize );
        break;
    }
    case RESP_INCORRECT_COMMAND:
    {
        break;
    }
    default:
    {
        stringstream ss;
        ss << "Don't know how to deal with rxMsg code: " << cmdToString(rxMsgCode);
        throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
    }
    }
}

LmsRxMsgPtr
parseBufferForRxMsgs( const uChar  *buffer,
                      int           bufferLength,
                      int          &bytesParsed )
{
    // cout<<"TRACE(messages.cpp): parseBufferForRxMsgs(length "<<bufferLength<<")" << endl;

    LmsRxMsgPtr rxMsg;
    int commandAndDataLength;
    int telegramLength;
    int msgStart;
    bytesParsed = 0;

    if ( bufferLength == 0 )
        return NULL;

    //
    // Hunt for the start of the message:
    //  - Look for (STX plus a valid checksum)/ACK/NACK
    //
    // This loop guarantees termination because an invalid message
    // always increments bytesParsed.
    //
    while ( true )
    {
        // Look for STX/ACK/NACK
        while ( true )
        {
            if ( buffer[bytesParsed] == STX )
            {
                break;
            }
            else if ( buffer[bytesParsed] == ACK ||
                      buffer[bytesParsed] == NACK )
            {
                // cout<<"TRACE(messages.cpp): parseBufferForRxMsgs(): found ACK or NACK -- returning." << endl;
                rxMsg = new LmsRxMsg;
                rxMsg->type = buffer[bytesParsed];
                bytesParsed++;
                return rxMsg;
            }
            else if ( bytesParsed >= bufferLength-1 )
            {
                return NULL;
            }

            // cout<<"TRACE(messages.cpp): Warn: Clearing out unexpected byte: "<<toHexString(buffer[bytesParsed])<< endl;

            bytesParsed++;
        }

        // cout<<"TRACE(messages.cpp): found possible msg start at buf pos " << bytesParsed << endl;

        if ( (bufferLength-bytesParsed) < MIN_RXMSG_SIZE ) 
        {
            // cout<<"TRACE(messages.cpp): Buffer too short to contain a message." << endl;
            return NULL;
        }

        msgStart = bytesParsed;

        commandAndDataLength = (buffer[msgStart+3] << 8) | (buffer[msgStart+2]);
        telegramLength = PREAMBLE_LENGTH + commandAndDataLength + CHECKSUM_LENGTH;
        // cout<<"TRACE(messages.cpp): telegramLength: " << telegramLength << endl;

        //
        // Quick check: is the length absurd?
        //
        if ( telegramLength > MAX_SICK_TELEGRAM_LENGTH )
        {
            // cout << "Telegram length of " << telegramLength << " exceeds max of " << MAX_SICK_TELEGRAM_LENGTH << ".  Must have found incorrect start of message."<<endl;
            bytesParsed++;
            continue;
        }

        //
        // Do we have the whole message?
        //
        if ( bufferLength-msgStart < telegramLength )
        {
            // cout<<"TRACE(messages.cpp): Telegram length is " << telegramLength << " bytes but so far we only have "<<bufferLength-msgStart<<" of them." << endl;
            return NULL;
        }

        //
        // Verify checksum
        //
        int incomingChecksum = 
            (buffer[msgStart+telegramLength-1] << 8) |
            buffer[msgStart+telegramLength-2];
        int computedChecksum = computeSickChecksum( &(buffer[msgStart]),
                                                    telegramLength-2 );
        bool checksumFailed = (incomingChecksum != computedChecksum);
        if ( checksumFailed )
        {
            IceUtil::Time t = IceUtil::Time::now();
            cout << "========== Checksum failed, timestamp: " << t.toDateTime() << endl;
            // cout << "WARN(messages.cpp): " << t.toDateTime() << ": Checksum failed at buf pos " <<bytesParsed << endl;
            // cout<<"TRACE(messages.cpp): checksum was over: " << toHexString( &(buffer[bytesParsed]), telegramLength ) << endl;
            bytesParsed++;
            // cout<<"TRACE(messages.cpp): returning after failed checksum, with bytesParsed = " << bytesParsed << endl;
            continue;
        }
        else
        {
            // Valid message!
            break;
        }
    }
    
    bytesParsed = msgStart+telegramLength;

    // cout<<"TRACE(messages.cpp): commandAndDataLength: " << commandAndDataLength << endl;

    rxMsg = new LmsRxMsg;
    rxMsg->type = buffer[msgStart+4];
    rxMsg->status = buffer[msgStart+telegramLength-3];

    try {
        parseRxMsg( rxMsg->type, &(buffer[msgStart+5]), commandAndDataLength-2, *rxMsg );
        return rxMsg;
    }
    catch ( const std::exception &e )
    {
        cout << __func__ << "(): Error parsing rxMsg: " << e.what() << endl;
        return NULL;
    }
}

void
constructRequestInstallationMode( std::vector<uChar> &commandAndData )
{
    const char* LMS_PASSWORD = "SICK_LMS";
    // const char* PLS_PASSWORD = "SICK_PLS";

    const int passwordLength = 8;
    commandAndData.resize( 2 + passwordLength );

    int pos=0;
    commandAndData[pos++] = CMD_SWITCH_OPERATING_MODE;
    commandAndData[pos++] = OPERATING_MODE_INSTALLATION;

    for ( int i=0; i < passwordLength; i++ )
    {
        commandAndData[pos++] = LMS_PASSWORD[i];
    }
}

void
constructRequestContinuousMode( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 2 );

    int pos=0;
    commandAndData[pos++] = CMD_SWITCH_OPERATING_MODE;
    commandAndData[pos++] = OPERATING_MODE_ALL_MEASURED_CONTINUOUS;
}

void
constructRequestMeasuredOnRequestMode( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 2 );

    int pos=0;
    commandAndData[pos++] = CMD_SWITCH_OPERATING_MODE;
    commandAndData[pos++] = OPERATING_MODE_MEASURED_ON_REQUEST;
}

void constructInitAndReset( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 1 );
    commandAndData[0] = CMD_INIT_AND_RESET;
}

void constructStatusRequest( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 1 );
    commandAndData[0] = CMD_REQUEST_LMS_STATUS;
}

void constructConfigurationRequest( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 1 );
    commandAndData[0] = CMD_REQUEST_LMS_CONFIGURATION;    
}

void constructConfigurationCommand( const LmsConfigurationData &c,
                                    std::vector<uChar> &commandAndData )
{
    commandAndData.resize(35);
    int pos=0;
    commandAndData[pos++] = CMD_CONFIGURE_LMS;

    commandAndData[pos++] = (unsigned char)(c.blanking & 0xff);
    commandAndData[pos++] = (unsigned char)((c.blanking>>8) & 0xff);

    commandAndData[pos++] = 0x70; // not used for lms 211/221/291
    commandAndData[pos++] = c.sensitivity;

    commandAndData[pos++] = c.availability;

    commandAndData[pos++] = c.measuringMode;
    commandAndData[pos++] = c.measuredValueUnit;
    commandAndData[pos++] = c.transientFieldSet;
    commandAndData[pos++] = c.subtractiveFields;   // 14
    commandAndData[pos++] = c.multipleEvaluation;
    commandAndData[pos++] = c.restart;
    commandAndData[pos++] = c.restartTime;
    commandAndData[pos++] = c.multipleEvaluationForSuppressed;
    commandAndData[pos++] = c.contourARef;
    commandAndData[pos++] = c.contourAPosToleranceBand; // 20
    commandAndData[pos++] = c.contourANegToleranceBand;
    commandAndData[pos++] = c.contourAStartAngle;
    commandAndData[pos++] = c.contourAStopAngle;
    commandAndData[pos++] = c.contourBRef;
    commandAndData[pos++] = c.contourBPosToleranceBand;
    commandAndData[pos++] = c.contourBNegToleranceBand;
    commandAndData[pos++] = c.contourBStartAngle;
    commandAndData[pos++] = c.contourBStopAngle;
    commandAndData[pos++] = c.contourARef2;
    commandAndData[pos++] = c.contourAPosToleranceBand2; // 30
    commandAndData[pos++] = c.contourCNegToleranceBand;
    commandAndData[pos++] = c.contourCStartAngle;
    commandAndData[pos++] = c.contourCStopAngle;
    commandAndData[pos++] = c.pixelOrientedEvaluation;
    commandAndData[pos++] = c.singleMeasuredValueEvaluation;

    commandAndData[pos++] = (unsigned char)(c.restartTimeFields & 0xff);
    commandAndData[pos++] = (unsigned char)((c.restartTimeFields>>8) & 0xff);
    commandAndData[pos++] = (unsigned char)(c.multipleEvaluationForDazzle & 0xff);
    commandAndData[pos++] = (unsigned char)((c.multipleEvaluationForDazzle>>8) & 0xff);

    assert( pos == (int)(commandAndData.size()) );
}

void constructRequestErrorMessage( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 1 );
    commandAndData[0] = CMD_REQUEST_ERROR_OR_TEST_MESSAGE;
}

void constructSwitchVariant( uint16_t scanningAngle,
                             uint16_t angularResolution,
                             std::vector<uChar> &commandAndData )                             
{
//     cout<<"TRACE(messages.cpp): configuring with scanning angle: " << scanningAngle << endl;
//     cout<<"TRACE(messages.cpp): angularResolution: " << angularResolution << endl;

    commandAndData.resize( 5 );
    commandAndData[0] = CMD_SWITCH_VARIANT;
    commandAndData[1] = (unsigned char)(scanningAngle & 0xff);
    commandAndData[2] = (unsigned char)((scanningAngle>>8) & 0xff);
    commandAndData[3] = (unsigned char)(angularResolution & 0xff);
    commandAndData[4] = (unsigned char)((angularResolution>>8) & 0xff);
}

void constructRequestOperatingDataCounter( std::vector<uChar> &commandAndData )
{
    commandAndData.resize( 3 );
    commandAndData[0] = CMD_REQUEST_OPERATING_DATA_COUNTER;    
    commandAndData[1] = 0;
    commandAndData[2] = 0;
}

void constructRequestBaudRate( std::vector<uChar> &commandAndData,
                               int baudRate )
{
    commandAndData.resize( 2 );
    commandAndData[0] = CMD_SWITCH_OPERATING_MODE;
    commandAndData[1] = baudRateIntToOperatingMode( baudRate );
}

}
