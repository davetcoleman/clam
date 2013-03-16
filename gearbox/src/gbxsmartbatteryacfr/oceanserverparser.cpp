/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <sstream>
#include <gbxutilacfr/tokenise.h>
#include <gbxsmartbatteryacfr/exceptions.h>
#include <gbxsmartbatteryacfr/smartbatteryparsing.h>

#include "oceanserverparser.h"

using namespace std;

namespace 
{
    // It is possible that binary characters enter the stream,
    // e.g. caused by electrical interference or dodgy serial ports.
    // This function checks for them
    bool
    containsBinaryCharacters( const std::string &line )
    {
        // check for binary characters
        for (unsigned int k=0; k<line.size()-2; k++)
        {            
            if ( iscntrl(line[k]) ) 
                return true;
        }
    
        return false;
    }
}

namespace gbxsmartbatteryacfr {

OceanServerParser::OceanServerParser( gbxutilacfr::Tracer &tracer )
    : tracer_(tracer)
{
}

void 
OceanServerParser::parseSystemData( const map<string,string> &keyValuePairs,
                                    OceanServerSystem        &batterySystem )
{   
    map<string,string>::const_iterator it;
    
    for (it=keyValuePairs.begin(); it!=keyValuePairs.end(); it++)
    {   
        if (it->first=="01") {
            batterySystem.setMinToEmpty( readMinutes(it->second) );
        }
        else if (it->first=="02") {
            // reserved, do nothing
        }
        else if (it->first=="03") {
            batterySystem.setMessageToSystem( it->second );
        }
        else if (it->first=="04") {
            batterySystem.setPercentCharge( readPercentByte(it->second) );
        }
        else 
        {
            stringstream ss;
            ss << "Unknown System key: " << it->first;
            throw ParsingException( ERROR_INFO, ss.str().c_str() );
        }
    }
}

void 
OceanServerParser::parseControllerData( const map<string,string> &keyValuePairs,
                                        OceanServerSystem        &batterySystem )
{
    map<string,string>::const_iterator it;
    
    for (it=keyValuePairs.begin(); it!=keyValuePairs.end(); it++)
    { 
        vector<bool> states;
        
        if (it->first=="01") {
            readFlags(it->second, states);
            batterySystem.availableBatteries() = states;
        }
        else if (it->first=="02") {
            readFlags(it->second, states);
            batterySystem.chargingStates() = states;
        }
        else if (it->first=="03") {
            readFlags(it->second, states);
            batterySystem.supplyingPowerStates() = states;
        }
        else if (it->first=="04") {
            // reserved, do nothing
        }
        else if (it->first=="05") {
            readFlags(it->second, states);
            batterySystem.chargePowerPresentStates() = states;
        }
        else if (it->first=="06") {
            readFlags(it->second, states);
            batterySystem.powerNoGoodStates() = states;
        }
        else if (it->first=="07") {
            readFlags(it->second, states);
            batterySystem.chargeInhibitedStates() = states;
        }
        else 
        {
            stringstream ss;
            ss << "Unknown controller key: " << it->first;
            throw ParsingException( ERROR_INFO, ss.str().c_str() );
        }

    }
    
}

void
OceanServerParser::parseSingleBatteryData( const map<string,string> &keyValuePairs, 
                                           unsigned int              batteryNum, 
                                           OceanServerSystem        &batterySystem )
{
    map<string,string>::const_iterator it;
    
    // get a reference to the battery whose fields we're updating
    SmartBattery &bat = batterySystem.battery( batteryNum );
    
    for (it=keyValuePairs.begin(); it!=keyValuePairs.end(); it++)
    {   
        SmartBatteryDataField smartField = keyToSmartField( it->first );
        
        switch( smartField )
        {
            case ManufacturerAccess: 
                bat.setManufacturerAccess( read16Flags( it->second ) ); break;
            case RemainingCapacityAlarm:
                bat.setRemainingCapacityAlarm( readCapacity( it->second ) ); break;
            case RemainingTimeAlarm:
                bat.setRemainingTimeAlarm( readMinutes( it->second ) ); break;
            case BatteryMode:
                bat.setBatteryMode( read16Flags( it->second ) ); break;
            case AtRate:
                bat.setAtRate( readRate( it->second ) ); break;
            case AtRateTimeToFull:
                bat.setAtRateTimeToFull( readMinutes( it->second ) ); break;
            case AtRateTimeToEmpty:
                bat.setAtRateTimeToEmpty( readMinutes( it->second ) ); break;
            case AtRateOk:
                bat.setAtRateOk( readRate( it->second ) ); break;
            case Temperature: 
                bat.setTemperature( readTemperature( it->second ) ); break;
            case Voltage:
                bat.setVoltage( readVoltage( it->second ) ); break;
            case Current: 
                bat.setCurrent( readCurrent( it->second ) ); break;
            case AverageCurrent:
                bat.setAverageCurrent( readCurrent( it->second ) ); break;
            case MaxError:
                bat.setMaxError( readPercentWord( it->second ) ); break;
            case RelativeStateOfCharge:
                bat.setRelativeStateOfCharge( readPercentWord( it->second ) ); break;
            case AbsoluteStateOfCharge:
                bat.setAbsoluteStateOfCharge( readPercentWord( it->second ) ); break;
            case RemainingCapacity:
                bat.setRemainingCapacity( readCapacity( it->second ) ); break;
            case FullChargeCapacity:
                bat.setFullChargeCapacity( readCapacity( it->second ) ); break;
            case RunTimeToEmpty:
                bat.setRunTimeToEmpty( readMinutes( it->second ) ); break;
            case AverageTimeToEmpty:
                bat.setAverageTimeToEmpty( readMinutes( it->second ) ); break;
            case AverageTimeToFull:
                bat.setAverageTimeToFull( readMinutes( it->second ) ); break;
            case ChargingCurrent:
                bat.setChargingCurrent( readCurrent( it->second ) ); break;
            case ChargingVoltage:
                bat.setChargingVoltage( readVoltage( it->second ) ); break;
            case BatteryStatus:
                bat.setBatteryStatus( read16Flags( it->second ) ); break;
            case CycleCount:
                bat.setCycleCount( readCount (it->second) ); break;
            case DesignCapacity:
                bat.setDesignCapacity( readCapacity( it->second ) ); break;
            case DesignVoltage:
                bat.setDesignVoltage( readVoltage( it->second ) ); break;
            case SpecificationInfo:
                bat.setSpecificationInfo( read16Flags( it->second ) ); break;
            case ManufactureDate:
                bat.setManufactureDate( read16Flags( it->second ) ); break;
            case SerialNumber:
                bat.setSerialNumber( readNumber( it->second ) ); break;
            case ManufacturerName:
                bat.setManufacturerName( it->second ); break;
            case DeviceName:
                bat.setDeviceName( it->second ); break;
            case DeviceChemistry:
                bat.setDeviceChemistry( it->second ); break;
            case ManufacturerData:
                bat.setManufacturerData( read16Flags( it->second ) ); break;
            case NUM_SMARTBATTERY_FIELDS:
            default:
                stringstream ss; ss << "Unknown Battery key: " << it->first;
                throw ParsingException( ERROR_INFO, ss.str().c_str() );
        }
    }
    
}

void 
OceanServerParser::parseFields( vector<string>    &fields, 
                                OceanServerSystem &batterySystem )
{
    if (fields.size()==0) return;
    
    // save the msgType string and remove from vector
    string msgType = fields[0];
    vector<string>::iterator it = fields.begin();
    fields.erase( it );
    
    // get the msg type key (S,C,B)
    const string &msgTypeKey = msgType.substr(0,2);
    
    // make key-value pairs
    map<string,string> keyValuePairs;    
    toKeyValuePairs( fields, keyValuePairs, tracer_ );
    
    if (msgTypeKey=="$S") 
    {
        parseSystemData( keyValuePairs, batterySystem );
    } 
    else if  (msgTypeKey=="$C")
    {
        parseControllerData( keyValuePairs, batterySystem );    
    }
    else if (msgTypeKey=="$B")
    {   
        stringstream ss(msgType.substr(3));
        int batteryNum;
        ss >> batteryNum;
        parseSingleBatteryData( keyValuePairs, batteryNum, batterySystem );
    }
    else
    {
        stringstream ss;
        ss << "Unknown message type: " << msgTypeKey;
        throw ParsingException( ERROR_INFO, ss.str().c_str() );
    }
}

bool 
OceanServerParser::atBeginningOfRecord( const std::string &line )
{
    vector<string> tokens = gbxutilacfr::tokenise( line, ",");
        
    if (tokens.size()>0) {
        if (tokens[0]!="$S") {
            return false;
        }
    }
    return true;
}


void
OceanServerParser::parse( vector<string>    &stringList, 
                          OceanServerSystem &batterySystem )
{
    // put the raw record into the batterySystem representation
    // useful for "higher-level" debugging: the caller can choose how to make use of this information
    batterySystem.rawRecord() = stringList;
        
    //
    // Debugging output
    //
    const int debugLevel = 10;
    if (tracer_.verbosity( gbxutilacfr::DebugTrace, gbxutilacfr::ToAny ) >= debugLevel)
    {
        stringstream ss;
        for (unsigned int i=0; i<stringList.size(); i++)
        {
            const string &str = stringList[i];
            ss << "Line: " << str;
            for (unsigned k=0; k<str.size()-2; k++)
            {
                unsigned int charValue = (unsigned int)str[k];
                ss << "Character (str/hex): " << str[k] << "/" << std::hex << charValue << endl;
            }
            ss << std::dec << endl;
        }
        ss << endl;
        tracer_.debug( ss.str(), debugLevel );
    }
    
    //
    // Parsing
    //
    for (unsigned int i=0; i<stringList.size(); i++)
    {
        if ( containsBinaryCharacters( stringList[i] ) )
             throw ParsingException( ERROR_INFO, "Found a binary character" );

        // divide the filteredString into 2 parts: data and checksum (if present)
        vector<string> checksumList = gbxutilacfr::tokenise( stringList[i], "%" );
        if ( checksumList.size()==2 )
        {        
            // we have a checksum, is it correct?
            if (!isChecksumValid( checksumList[0], checksumList[1] ) )
                throw ParsingException( ERROR_INFO, "Checksum failed!" );
        }
                 
        // divide the data into individual fields and parse
        if (checksumList.size()==0)
            throw ParsingException( ERROR_INFO, "String length is 0" );
        vector<string> fields = gbxutilacfr::tokenise( checksumList[0], "," );
        parseFields( fields, batterySystem );
    }

}

}

