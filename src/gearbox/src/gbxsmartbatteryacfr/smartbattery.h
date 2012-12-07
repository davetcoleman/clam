/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_SMARTBATTERY_H
#define GBX_SMARTBATTERY_H

#include <stdint.h>

#include <vector>
#include <string>
#include <assert.h>

namespace gbxsmartbatteryacfr {

//! Smart battery data specification
//! Specs can be found at http://sbs-forum.org/specs/
enum SmartBatteryDataField 
{
    ManufacturerAccess = 0,
    RemainingCapacityAlarm,
    RemainingTimeAlarm,
    BatteryMode,
    AtRate,
    AtRateTimeToFull,
    AtRateTimeToEmpty,
    AtRateOk,
    Temperature,
    Voltage,
    Current,
    AverageCurrent,
    MaxError,
    RelativeStateOfCharge,
    AbsoluteStateOfCharge,
    RemainingCapacity,
    FullChargeCapacity,
    RunTimeToEmpty,
    AverageTimeToEmpty,
    AverageTimeToFull,
    ChargingCurrent,
    ChargingVoltage,
    BatteryStatus,
    CycleCount,
    DesignCapacity,
    DesignVoltage,
    SpecificationInfo,
    ManufactureDate,
    SerialNumber,
    ManufacturerName,
    DeviceName,
    DeviceChemistry,
    ManufacturerData,
    NUM_SMARTBATTERY_FIELDS
};
    
//! Converts a key (string) to a SmartBatteryDataField.
//! Throws a ParsingException if key is unknown.
SmartBatteryDataField keyToSmartField( const std::string &key );
    
//! SmartBattery class holds all the data of a single smart battery.
//! Since not all data is always present, access to data needs to be done as follows:
//! if (smartBattery.has(Temperature)) { double myTemp = smartBattery.temperature(); }
class SmartBattery 
{
    public:
        SmartBattery() : has_(NUM_SMARTBATTERY_FIELDS)
        { std::fill( has_.begin(), has_.end(), false ); };
        
        bool has( SmartBatteryDataField field ) const;
        
        uint16_t manufacturerAccess() const { assert(has_[ManufacturerAccess]); return manufacturerAccess_; };
        void setManufacturerAccess( uint16_t manufacturerAccess ) { has_[ManufacturerAccess] = true; manufacturerAccess_ = manufacturerAccess; };       
         
        int remainingCapacityAlarm() const { assert(has_[RemainingCapacityAlarm]); return remainingCapacityAlarm_; };
        void setRemainingCapacityAlarm( int remainingCapacityAlarm ) { has_[RemainingCapacityAlarm] = true; remainingCapacityAlarm_ = remainingCapacityAlarm; };
        
        int remainingTimeAlarm() const { assert(has_[RemainingTimeAlarm]); return remainingTimeAlarm_; };
        void setRemainingTimeAlarm( int remainingTimeAlarm ) { has_[RemainingTimeAlarm] = true; remainingTimeAlarm_ = remainingTimeAlarm; };     
        
        uint16_t batteryMode() const { assert(has_[BatteryMode]); return batteryMode_; };
        void setBatteryMode( uint16_t batteryMode ) { has_[BatteryMode] = true; batteryMode_ = batteryMode; };       
         
        int atRate() const { assert(has_[AtRate]); return atRate_; };
        void setAtRate( int atRate ) { has_[AtRate] = true; atRate_ = atRate; };         
        
        int atRateTimeToFull() const { assert(has_[AtRateTimeToFull]); return atRateTimeToFull_; };
        void setAtRateTimeToFull( int atRateTimeToFull ) { has_[AtRateTimeToFull] = true; atRateTimeToFull_ = atRateTimeToFull; };  
           
        int atRateTimeToEmpty() const { assert(has_[AtRateTimeToEmpty]); return atRateTimeToEmpty_; };
        void setAtRateTimeToEmpty( int atRateTimeToEmpty ) { has_[AtRateTimeToEmpty] = true; atRateTimeToEmpty_ = atRateTimeToEmpty; };  
           
        bool atRateOk() const { assert(has_[AtRateOk]); return atRateOk_; };
        void setAtRateOk( bool atRateOk ) { has_[AtRateOk] = true; atRateOk_ = atRateOk; };  
        
        double temperature() const { assert(has_[Temperature]); return temperature_; };
        void setTemperature( double temperature ) { has_[Temperature] = true; temperature_ = temperature; };
        
        double voltage() const { assert(has_[Voltage]); return voltage_; };
        void setVoltage( double voltage ) { has_[Voltage] = true; voltage_ = voltage; };
        
        double current() const { assert(has_[Current]); return current_; };
        void setCurrent( double current ) { has_[Current] = true; current_ = current; };
        
        double averageCurrent() const { assert(has_[AverageCurrent]); return averageCurrent_; };
        void setAverageCurrent( double averageCurrent ) { has_[AverageCurrent] = true; averageCurrent_ = averageCurrent; };
        
        int maxError() const { assert(has_[MaxError]); return maxError_; };
        void setMaxError( int maxError ) { has_[MaxError] = true; maxError_ = maxError; };    
              
        int relativeStateOfCharge() const { assert(has_[RelativeStateOfCharge]); return relativeStateOfCharge_; };
        void setRelativeStateOfCharge( int relativeStateOfCharge ) { has_[RelativeStateOfCharge] = true; relativeStateOfCharge_ = relativeStateOfCharge; };  
                
        int absoluteStateOfCharge() const { assert(has_[AbsoluteStateOfCharge]); return absoluteStateOfCharge_; };
        void setAbsoluteStateOfCharge( int absoluteStateOfCharge ) { has_[AbsoluteStateOfCharge] = true; absoluteStateOfCharge_ = absoluteStateOfCharge; };  
        
        int remainingCapacity() const { assert(has_[RemainingCapacity]); return remainingCapacity_; };
        void setRemainingCapacity( int remainingCapacity ) { has_[RemainingCapacity] = true; remainingCapacity_ = remainingCapacity; };      
          
        int fullChargeCapacity() const { assert(has_[FullChargeCapacity]); return fullChargeCapacity_; };
        void setFullChargeCapacity( int fullChargeCapacity ) { has_[FullChargeCapacity] = true; fullChargeCapacity_ = fullChargeCapacity; };
               
        int runTimeToEmpty() const { assert(has_[RunTimeToEmpty]); return runTimeToEmpty_; };
        void setRunTimeToEmpty( int runTimeToEmpty ) { has_[RunTimeToEmpty] = true; runTimeToEmpty_ = runTimeToEmpty; };                
         
        int averageTimeToEmpty() const { assert(has_[AverageTimeToEmpty]); return averageTimeToEmpty_; };
        void setAverageTimeToEmpty( int averageTimeToEmpty ) { has_[AverageTimeToEmpty] = true; averageTimeToEmpty_ = averageTimeToEmpty; };  
           
        int averageTimeToFull() const { assert(has_[AverageTimeToFull]); return averageTimeToFull_; };
        void setAverageTimeToFull( int averageTimeToFull ) { has_[AverageTimeToFull] = true; averageTimeToFull_ = averageTimeToFull; };  
            
        double chargingCurrent() const { assert(has_[ChargingCurrent]); return chargingCurrent_; };
        void setChargingCurrent( double chargingCurrent ) { has_[ChargingCurrent] = true; chargingCurrent_ = chargingCurrent; };      
            
        double chargingVoltage() const { assert(has_[ChargingVoltage]); return chargingVoltage_; };
        void setChargingVoltage( double chargingVoltage ) { has_[ChargingVoltage] = true; chargingVoltage_ = chargingVoltage; };      
   
        uint16_t batteryStatus() const { assert(has_[BatteryStatus]); return batteryStatus_; };
        void setBatteryStatus( uint16_t batteryStatus ) { has_[BatteryStatus] = true; batteryStatus_ = batteryStatus; };       
             
        int cycleCount() const { assert(has_[CycleCount]); return cycleCount_; };
        void setCycleCount( int cycleCount ) { has_[CycleCount] = true; cycleCount_ = cycleCount; };  
             
        int designCapacity() const { assert(has_[DesignCapacity]); return designCapacity_; };
        void setDesignCapacity( int designCapacity ) { has_[DesignCapacity] = true; designCapacity_ = designCapacity; };      
               
        double designVoltage() const { assert(has_[DesignVoltage]); return designVoltage_; };
        void setDesignVoltage( double designVoltage ) { has_[DesignVoltage] = true; designVoltage_ = designVoltage; };   
           
        uint16_t specificationInfo() const { assert(has_[SpecificationInfo]); return specificationInfo_; };
        void setSpecificationInfo( uint16_t specificationInfo ) { has_[SpecificationInfo] = true; specificationInfo_ = specificationInfo; };       
            
        uint16_t manufactureDate() const { assert(has_[ManufactureDate]); return manufactureDate_; };
        void setManufactureDate ( uint16_t manufactureDate ) { has_[ManufactureDate] = true; manufactureDate_ = manufactureDate; };       
              
        int serialNumber() const { assert(has_[SerialNumber]); return serialNumber_; };
        void setSerialNumber( int serialNumber ) { has_[SerialNumber] = true; serialNumber_ = serialNumber; };  
        
        const std::string manufacturerName() const { assert(has_[ManufacturerName]); return manufacturerName_; };
        void setManufacturerName( std::string manufacturerName ) { has_[ManufacturerName] = true;  manufacturerName_ = manufacturerName; };
           
        const std::string deviceName() const { assert(has_[DeviceName]); return deviceName_; };
        void setDeviceName( std::string deviceName ) { has_[DeviceName] = true;  deviceName_ = deviceName; };
             
        const std::string deviceChemistry() const { assert(has_[DeviceChemistry]); return deviceChemistry_; };
        void setDeviceChemistry( std::string deviceChemistry ) { has_[DeviceChemistry] = true;  deviceChemistry_ = deviceChemistry; };
                  
        uint16_t manufacturerData() const { assert(has_[ManufacturerData]); return manufacturerData_; };
        void setManufacturerData( uint16_t manufacturerData ) { has_[ManufacturerData] = true; manufacturerData_ = manufacturerData; };       
      
    private:
        
        std::vector<bool> has_;
        
        uint16_t manufacturerAccess_;       // manufacturer specific
        int remainingCapacityAlarm_;        // mAh or 10 mWh
        int remainingTimeAlarm_;            // min
        uint16_t batteryMode_;              // flags, see specs
        int atRate_;                        // mA or 10 mW
        int atRateTimeToFull_;              // min
        int atRateTimeToEmpty_;             // min
        bool atRateOk_;                     // bool
        double temperature_;                // Celsius
        double voltage_;                    // V
        double current_;                    // A
        double averageCurrent_;             // A
        int maxError_;                      // percent
        int relativeStateOfCharge_;         // percent
        int absoluteStateOfCharge_;         // percent
        int remainingCapacity_;             // mAh or 10 mWh
        int fullChargeCapacity_;            // mAh or 10 mWh
        int runTimeToEmpty_;                // min
        int averageTimeToEmpty_;            // min
        int averageTimeToFull_;             // min
        double chargingCurrent_;            // A
        double chargingVoltage_;            // V
        uint16_t batteryStatus_;            // flags, see specs
        int cycleCount_;                    // count
        int designCapacity_;                // mAh or 10 mWh
        double designVoltage_;              // V
        uint16_t specificationInfo_;        // flags, see specs
        uint16_t manufactureDate_;          // raw, see specs
        int serialNumber_;                  // number
        std::string manufacturerName_;
        std::string deviceName_;
        std::string deviceChemistry_;
        uint16_t manufacturerData_;         // manufacturer specific
        
        
};

//! Puts SmartBattery data into a human-readable string
std::string toString( const SmartBattery &b );

//! Puts SmartBattery data into a machine-readable ASCII string
std::string toLogString( const SmartBattery &b );

}

#endif

