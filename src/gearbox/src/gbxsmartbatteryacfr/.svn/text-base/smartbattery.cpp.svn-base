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
#include <gbxsmartbatteryacfr/exceptions.h>

#include "smartbattery.h"

using namespace std;

namespace gbxsmartbatteryacfr {

SmartBatteryDataField keyToSmartField( const string &key )
{
    if  (key=="00") return ManufacturerAccess;
    else if (key=="01") return RemainingCapacityAlarm;
    else if (key=="02") return RemainingTimeAlarm;
    else if (key=="03") return BatteryMode;
    else if (key=="04") return AtRate;
    else if (key=="05") return AtRateTimeToFull;
    else if (key=="06") return AtRateTimeToEmpty;
    else if (key=="07") return AtRateOk;
    else if (key=="08") return Temperature;
    else if (key=="09") return Voltage;
    else if (key=="0A") return Current;
    else if (key=="0B") return AverageCurrent;
    else if (key=="0C") return MaxError;
    else if (key=="0D") return RelativeStateOfCharge;
    else if (key=="0E") return AbsoluteStateOfCharge;
    else if (key=="0F") return RemainingCapacity;
    else if (key=="10") return FullChargeCapacity;
    else if (key=="11") return RunTimeToEmpty;
    else if (key=="12") return AverageTimeToEmpty;
    else if (key=="13") return AverageTimeToFull;
    else if (key=="14") return ChargingCurrent;
    else if (key=="15") return ChargingVoltage;
    else if (key=="16") return BatteryStatus;
    else if (key=="17") return CycleCount;
    else if (key=="18") return DesignCapacity;
    else if (key=="19") return DesignVoltage;
    else if (key=="1A") return SpecificationInfo;
    else if (key=="1B") return ManufactureDate;
    else if (key=="1C") return SerialNumber;
    else if (key=="20") return ManufacturerName;
    else if (key=="21") return DeviceName;
    else if (key=="22") return DeviceChemistry;
    else if (key=="23") return ManufacturerData;
    else 
    {
        stringstream ss;
        ss << "Unknown field: " << key;
        throw ParsingException( ERROR_INFO, ss.str().c_str() );
    }
}

string toString( const SmartBattery &b )
{    
    stringstream ss;
    
    if ( b.has( ManufacturerAccess ) )
        ss << "ManufacturerAccess (16 bits):          " << std::hex << b.manufacturerAccess() << endl;
    if ( b.has( RemainingCapacityAlarm ) )
        ss << "RemainingCapacityAlarm (mAh or 10mWh): " << std::dec << b.remainingCapacityAlarm() << endl;
    if ( b.has( RemainingTimeAlarm ) )
        ss << "RemainingTimeAlarm (min):              " << std::dec << b.remainingTimeAlarm() << endl;
    if ( b.has( BatteryMode ) )
        ss << "BatteryMode (16 bits):                 " << std::hex << b.batteryMode() << endl;
    if ( b.has( AtRate ) )
        ss << "AtRate (mA or 10mW):                   " << std::dec << b.atRate() << endl;
    if ( b.has( AtRateTimeToFull ) )
        ss << "AtRateTimeToFull (min):                " << std::dec << b.atRateTimeToFull() << endl;
    if ( b.has( AtRateTimeToEmpty ) )
        ss << "AtRateTimeToEmpty (min):               " << std::dec << b.atRateTimeToEmpty() << endl;
    if ( b.has( AtRateOk ) )
        ss << "AtRateOk (bool):                       " << std::dec << b.atRateOk() << endl;
    if ( b.has( Temperature ) )
        ss << "Temperature (degC):                    " << std::dec << b.temperature() << endl;
    if ( b.has( Voltage ) )
        ss << "Voltage (V):                           " << std::dec << b.voltage() << endl;
    if ( b.has( Current ) )
        ss << "Current (A):                           " << std::dec << b.current() << endl;
    if ( b.has( AverageCurrent ) )
        ss << "AverageCurrent (A):                    " << std::dec << b.averageCurrent() << endl;
    if ( b.has( MaxError ) )
        ss << "MaxError (%):                          " << std::dec << b.maxError() << endl;
    if ( b.has( RelativeStateOfCharge ) )
        ss << "RelativeStateOfCharge (%):             " << std::dec << b.relativeStateOfCharge() << endl;
    if ( b.has( AbsoluteStateOfCharge ) )
        ss << "AbsoluteStateOfCharge (%):             " << std::dec << b.absoluteStateOfCharge() << endl;
    if ( b.has( RemainingCapacity ) )
        ss << "RemainingCapacity (mAh or 10 mWh):     " << std::dec << b.remainingCapacity() << endl;
    if ( b.has( FullChargeCapacity ) )
        ss << "FullChargeCapacity (mAh or 10 mWh):    " << std::dec << b.fullChargeCapacity() << endl;
    if ( b.has( RunTimeToEmpty ) )
        ss << "RunTimeToEmpty (min):                  " << std::dec << b.runTimeToEmpty() << endl;
    if ( b.has( AverageTimeToEmpty ) )
        ss << "AverageTimeToEmpty (min):              " << std::dec << b.averageTimeToEmpty() << endl;
    if ( b.has( AverageTimeToFull ) )
        ss << "AverageTimeToFull (min):               " << std::dec << b.averageTimeToFull() << endl;
    if ( b.has( ChargingCurrent ) )
        ss << "ChargingCurrent (A):                   " << std::dec << b.chargingCurrent() << endl;
    if ( b.has( ChargingVoltage ) )
        ss << "ChargingVoltage (V):                   " << std::dec << b.chargingVoltage() << endl;
    if ( b.has( BatteryStatus ) )
        ss << "BatteryStatus (16 bits):               " << std::hex << b.batteryStatus() << endl;
    if ( b.has( CycleCount ) )
        ss << "CycleCount (number):                   " << std::dec << b.cycleCount() << endl;
    if ( b.has( DesignCapacity ) )
        ss << "DesignCapacity (mAh or 10 mWh):        " << std::dec << b.designCapacity() << endl;
    if ( b.has( DesignVoltage ) )
        ss << "DesignVoltage (V):                     " << std::dec << b.designVoltage() << endl;
    if ( b.has( SpecificationInfo ) )
        ss << "SpecificationInfo (16 bits):           " << std::hex <<  b.specificationInfo() << endl;
    if ( b.has( ManufactureDate ) )
        ss << "ManufactureDate (16 bits):             " << std::hex <<  b.manufactureDate() << endl;
    if ( b.has( SerialNumber ) )
        ss << "SerialNumber (number):                 " << std::dec << b.serialNumber() << endl;
    if ( b.has( ManufacturerName ) )
        ss << "ManufacturerName:                      " << b.manufacturerName() << endl;
    if ( b.has( DeviceName ) )
        ss << "DeviceName:                            " << b.deviceName() << endl;
    if ( b.has( DeviceChemistry ) )
        ss << "DeviceChemistry:                       " << b.deviceChemistry() << endl;
    if ( b.has( ManufacturerData ) )
        ss << "ManufacturerData (16 bits):            " << std::hex << b.manufacturerData() << endl;
    
    return ss.str();
}

// For logging, we want a data entry per field, so it's easier to parse.
// For cases where data is not available (NA), we define "special" data entries for each type.
// This is a bit sketchy but ok for the purpose of simpler logging.
const int NA_DEC = -999;
const uint16_t NA_HEX = 11111;
const std::string NA_STR = "XXXX";

string toLogString( const SmartBattery &b )
{    
    stringstream ss;
    
    if ( b.has( ManufacturerAccess ) ) ss << std::hex << b.manufacturerAccess(); 
    else ss << std::hex << NA_HEX; ss << " ";
    
    if ( b.has( RemainingCapacityAlarm ) ) ss << std::dec << b.remainingCapacityAlarm();
    else ss << std::dec << NA_DEC; ss << " ";
        
    if ( b.has( RemainingTimeAlarm ) ) ss << std::dec << b.remainingTimeAlarm();
    else ss << std::dec << NA_DEC; ss << " ";
        
    if ( b.has( BatteryMode ) ) ss << std::hex << b.batteryMode();
    else ss << std::hex << NA_HEX; ss << " ";
        
    if ( b.has( AtRate ) ) ss << std::dec << b.atRate();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( AtRateTimeToFull ) ) ss << std::dec << b.atRateTimeToFull();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( AtRateTimeToEmpty ) ) ss << std::dec << b.atRateTimeToEmpty();
    else ss << std::dec << NA_DEC; ss << " ";
        
    if ( b.has( AtRateOk ) ) ss << std::dec << b.atRateOk();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( Temperature ) ) ss << std::dec << b.temperature();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( Voltage ) ) ss << std::dec << b.voltage();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( Current ) ) ss << std::dec << b.current();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( AverageCurrent ) ) ss << std::dec << b.averageCurrent();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( MaxError ) ) ss << std::dec << b.maxError();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( RelativeStateOfCharge ) ) ss << std::dec << b.relativeStateOfCharge();
    else ss << std::dec << NA_DEC; ss << " ";
    
    if ( b.has( AbsoluteStateOfCharge ) ) ss << std::dec << b.absoluteStateOfCharge();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( RemainingCapacity ) ) ss << std::dec << b.remainingCapacity();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( FullChargeCapacity ) ) ss << std::dec << b.fullChargeCapacity();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( RunTimeToEmpty ) ) ss << std::dec << b.runTimeToEmpty();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( AverageTimeToEmpty ) ) ss << std::dec << b.averageTimeToEmpty();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( AverageTimeToFull ) ) ss << std::dec << b.averageTimeToFull();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( ChargingCurrent ) ) ss << std::dec << b.chargingCurrent();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( ChargingVoltage ) ) ss << std::dec << b.chargingVoltage();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( BatteryStatus ) ) ss << std::hex << b.batteryStatus();
    else ss << std::hex << NA_HEX; ss << " ";
    
    if ( b.has( CycleCount ) ) ss << std::dec << b.cycleCount();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( DesignCapacity ) ) ss << std::dec << b.designCapacity();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( DesignVoltage ) ) ss << std::dec << b.designVoltage();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( SpecificationInfo ) ) ss << std::hex <<  b.specificationInfo();
    else ss << std::hex << NA_HEX; ss << " ";
    
    if ( b.has( ManufactureDate ) ) ss << std::hex <<  b.manufactureDate();
    else ss << std::hex << NA_HEX; ss << " ";
    
    if ( b.has( SerialNumber ) ) ss << std::dec << b.serialNumber();
    else ss << std::dec << NA_DEC; ss << " ";

    if ( b.has( ManufacturerName ) ) ss << b.manufacturerName();
    else ss << std::dec << NA_STR; ss << " ";
    
    if ( b.has( DeviceName ) ) ss << b.deviceName();
    else ss << std::dec << NA_STR; ss << " ";
    
    if ( b.has( DeviceChemistry ) ) ss << b.deviceChemistry();
    else ss << std::dec << NA_STR; ss << " ";
    
    if ( b.has( ManufacturerData ) ) ss << std::hex << b.manufacturerData();
    else ss << std::hex << NA_HEX;
               
    return ss.str();
}

bool
SmartBattery::has( SmartBatteryDataField field ) const 
{ 
    if ( (int)field >= (int)(has_.size()) ) {
        cout << "has(): field=" << field << ", has_.size=" << has_.size() << endl;
    }
    assert( (int)field < (int)(has_.size()) ); 
    return has_[field]; 
};

}
