/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <sstream>
#include <cstring>
#include <gbxsmartbatteryacfr/exceptions.h>

#include "oceanserversystem.h"

using namespace std;

namespace gbxsmartbatteryacfr {
    
//
// Helper functions    
//
namespace {
    
string toString( const vector<bool> &flags )
{
    stringstream ss;
    for (unsigned int i=0; i<flags.size(); i++)
    {
        ss << flags[i] << " ";
    }
    return ss.str();
}

string toLogString( const vector<bool> &flags )
{
    stringstream ss;
    for (unsigned int i=0; i<flags.size(); i++)
    {
        ss << flags[i];
    }
    return ss.str();
}

}
    
    
//
// Non-member functions
//

string toString( const OceanServerSystem &system )
{
    stringstream ss;
    ss << "Charge:          \t" << system.percentCharge() << endl;
    ss << "Minutes to empty:\t" << system.minToEmpty() << endl;
    ss << "Available batt.: \t" << toString( system.availableBatteries() ) << endl;
    ss << "Charging:        \t" << toString( system.chargingStates() ) << endl;
    ss << "Supplying power: \t" << toString( system.supplyingPowerStates() ) << endl;
    ss << "Charge power:    \t" << toString( system.chargePowerPresentStates() ) << endl;
    ss << "Power no good:   \t" << toString( system.powerNoGoodStates() ) << endl;
    ss << "Charge inhibited:\t" << toString( system.chargeInhibitedStates() ) << endl;
    
    map<int,SmartBattery>::const_iterator it;
    for (it=system.batteries().begin(); it!=system.batteries().end(); it++)
    {
        ss << "Data from battery number: " << it->first << endl;
        ss << toString( it->second ) << endl;
    }

    return ss.str();
}
    
string toLogString( const OceanServerSystem &system )
{
    stringstream ss;
    ss << system.percentCharge() << " ";
    ss << system.minToEmpty() << " ";
    ss << toLogString( system.availableBatteries() ) << " ";
    ss << toLogString( system.chargingStates() ) << " ";
    ss << toLogString( system.supplyingPowerStates() ) << " ";
    ss << toLogString( system.chargePowerPresentStates() ) << " ";
    ss << toLogString( system.powerNoGoodStates() ) << " ";
    ss << toLogString( system.chargeInhibitedStates() ) << endl;
    
    ss << system.batteries().size();
    
    map<int,SmartBattery>::const_iterator it;
    for (it=system.batteries().begin(); it!=system.batteries().end(); it++)
    {
        ss << it->first << " " << toLogString( it->second ) << endl;
    }

    return ss.str();
}
    
void updateWithNewData( const OceanServerSystem &from, 
                        OceanServerSystem       &to )
{
    to.rawRecord() = from.rawRecord();
    
    typedef map<int,SmartBattery>::const_iterator BatIt;
    
    to.setPercentCharge( from.percentCharge() );
    to.setMinToEmpty( from.minToEmpty() );
    to.setMessageToSystem( from.messageToSystem() );
    to.availableBatteries() = from.availableBatteries();
    to.chargingStates() = from.chargingStates();
    to.supplyingPowerStates() = from.supplyingPowerStates();
    to.chargePowerPresentStates() = from.chargePowerPresentStates();
    to.powerNoGoodStates() = from.powerNoGoodStates();
    to.chargeInhibitedStates() = from.chargeInhibitedStates();
    
    for (BatIt it=from.batteries().begin(); it!=from.batteries().end(); it++)
    {        
        const SmartBattery &fromB = from.battery( it->first );
        SmartBattery &toB = to.battery( it->first ); 
        
        if ( fromB.has( ManufacturerAccess  ) ) toB.setManufacturerAccess ( fromB.manufacturerAccess () );
        if ( fromB.has( RemainingCapacityAlarm  ) ) toB.setRemainingCapacityAlarm ( fromB.remainingCapacityAlarm () );
        if ( fromB.has( RemainingTimeAlarm  ) ) toB.setRemainingTimeAlarm ( fromB.remainingTimeAlarm () );
        if ( fromB.has( BatteryMode  ) ) toB.setBatteryMode ( fromB.batteryMode () );
        if ( fromB.has( AtRate  ) ) toB.setAtRate ( fromB.atRate () );
        if ( fromB.has( AtRateTimeToFull  ) ) toB.setAtRateTimeToFull ( fromB.atRateTimeToFull () );
        if ( fromB.has( AtRateTimeToEmpty  ) ) toB.setAtRateTimeToEmpty ( fromB.atRateTimeToEmpty () );
        if ( fromB.has( AtRateOk  ) ) toB.setAtRateOk ( fromB.atRateOk () );
        if ( fromB.has( Temperature  ) ) toB.setTemperature ( fromB.temperature () );
        if ( fromB.has( Voltage  ) ) toB.setVoltage ( fromB.voltage () );
        if ( fromB.has( Current  ) ) toB.setCurrent ( fromB.current () );
        if ( fromB.has( AverageCurrent  ) ) toB.setAverageCurrent ( fromB.averageCurrent () );
        if ( fromB.has( MaxError  ) ) toB.setMaxError ( fromB.maxError () );
        if ( fromB.has( RelativeStateOfCharge  ) ) toB.setRelativeStateOfCharge ( fromB.relativeStateOfCharge () );
        if ( fromB.has( AbsoluteStateOfCharge  ) ) toB.setAbsoluteStateOfCharge ( fromB.absoluteStateOfCharge () );
        if ( fromB.has( RemainingCapacity  ) ) toB.setRemainingCapacity ( fromB.remainingCapacity () );
        if ( fromB.has( FullChargeCapacity  ) ) toB.setFullChargeCapacity ( fromB.fullChargeCapacity () );
        if ( fromB.has( RunTimeToEmpty  ) ) toB.setRunTimeToEmpty ( fromB.runTimeToEmpty () );
        if ( fromB.has( AverageTimeToEmpty  ) ) toB.setAverageTimeToEmpty ( fromB.averageTimeToEmpty () );
        if ( fromB.has( AverageTimeToFull  ) ) toB.setAverageTimeToFull ( fromB.averageTimeToFull () );
        if ( fromB.has( ChargingCurrent  ) ) toB.setChargingCurrent ( fromB.chargingCurrent () );
        if ( fromB.has( ChargingVoltage  ) ) toB.setChargingVoltage ( fromB.chargingVoltage () );
        if ( fromB.has( BatteryStatus  ) ) toB.setBatteryStatus ( fromB.batteryStatus () );
        if ( fromB.has( CycleCount  ) ) toB.setCycleCount ( fromB.cycleCount () );
        if ( fromB.has( DesignCapacity  ) ) toB.setDesignCapacity ( fromB.designCapacity () );
        if ( fromB.has( DesignVoltage  ) ) toB.setDesignVoltage ( fromB.designVoltage () );
        if ( fromB.has( SpecificationInfo  ) ) toB.setSpecificationInfo ( fromB.specificationInfo () );
        if ( fromB.has( ManufactureDate  ) ) toB.setManufactureDate ( fromB.manufactureDate () );
        if ( fromB.has( SerialNumber  ) ) toB.setSerialNumber ( fromB.serialNumber () );
        if ( fromB.has( ManufacturerName  ) ) toB.setManufacturerName ( fromB.manufacturerName () );
        if ( fromB.has( DeviceName  ) ) toB.setDeviceName ( fromB.deviceName () );
        if ( fromB.has( DeviceChemistry  ) ) toB.setDeviceChemistry ( fromB.deviceChemistry () );
        if ( fromB.has( ManufacturerData  ) ) toB.setManufacturerData ( fromB.manufacturerData () );
    }
    
    // check if reaping needs to be done, if not return
    if ( from.batteries().size() == to.batteries().size() ) 
        return;
  
    // store battery ids from batteries which need to be reaped in a vector
    vector<int> reapingIds;

    // go through all 'to' batteries and check if they are also in 'from'
    for (BatIt it=to.batteries().begin(); it!=to.batteries().end(); it++)
    {
        const int batId = it->first;
        
        BatIt itFrom = from.batteries().find( batId );
        if ( itFrom == from.batteries().end() ) {
            // battery is in 'to' but not in 'from' -> needs to be reaped
            reapingIds.push_back( batId );
        }
    }
    
    // reap batteries
    for (unsigned int i=0; i<reapingIds.size(); i++) {
        to.eraseBattery( reapingIds[i] );
    }
    
}

bool isChargePowerPresent( const gbxsmartbatteryacfr::OceanServerSystem &batterySystem )
{
    for (unsigned int i=0; i<batterySystem.chargePowerPresentStates().size(); ++i)
    {
        if (batterySystem.chargePowerPresentStates()[i]==true)
            return true;
    }

    return false;
}


//
// Member functions
//
OceanServerSystem::OceanServerSystem()
    : isEmpty_(true),
      percentCharge_(-1),
      minToEmpty_(-1),
      messageToSystem_("")
{
    // fixed number of slots for oceanserver system
    const int NUM_BATTERY_SLOTS = 8;
    availableBatteries_.resize(NUM_BATTERY_SLOTS);
    chargingStates_.resize(NUM_BATTERY_SLOTS);
    supplyingPowerStates_.resize(NUM_BATTERY_SLOTS);
    chargePowerPresentStates_.resize(NUM_BATTERY_SLOTS);
    powerNoGoodStates_.resize(NUM_BATTERY_SLOTS);
    chargeInhibitedStates_.resize(NUM_BATTERY_SLOTS);
}

// read access to all batteries
const map<int,SmartBattery>&
OceanServerSystem::batteries() const
{ 
    return batteries_; 
}

// write access to single battery
SmartBattery& 
OceanServerSystem::battery( unsigned int batteryNumber )
{
    isEmpty_=false;
    
    map<int,SmartBattery>::iterator it = batteries_.find(batteryNumber);
    if ( it==batteries_.end() )
    {
        // we don't have it, so instantiate a new one
        SmartBattery b;
        batteries_[batteryNumber] = b;
        return batteries_[batteryNumber];
    }
    
    return it->second;
}  

// read access to single battery
const SmartBattery& 
OceanServerSystem::battery( unsigned int batteryNumber ) const
{
    map<int,SmartBattery>::const_iterator it = batteries_.find(batteryNumber);
    if ( it==batteries_.end() )
    {
        stringstream ss; 
        ss << "Trying to read from non-existent battery " << batteryNumber;
        throw ParsingException( ERROR_INFO, ss.str().c_str() );
    }
    return it->second;
}

void
OceanServerSystem::eraseBattery( unsigned int batteryNumber )
{
    batteries_.erase( batteries_.find( batteryNumber ) );    
}

} //namespace
