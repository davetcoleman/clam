/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_OCEANSERVER_SYSTEM_H
#define GBX_OCEANSERVER_SYSTEM_H

#include <map>
#include <gbxsmartbatteryacfr/smartbattery.h>

namespace gbxsmartbatteryacfr
{

//!
//! Class representing the OceanServer battery system data
//! Contains average values of the whole system and values from individual batteries
//!
//! @author Tobias Kaupp
//!
class OceanServerSystem
{
    public:

        //! Initialises all data in OceanServerSystem class
        OceanServerSystem();

        //! Returns true if no valid data has been set
        bool isEmpty() const { return isEmpty_; };
        
        //! Read access to all batteries
        const std::map<int,SmartBattery>& batteries() const;
        
        //! Easy write access to single battery, instantiates a new one if it doesn't exist
        SmartBattery& battery( unsigned int batteryNumber );   
        
        //! Easy read access to single battery, battery must exist
        const SmartBattery& battery( unsigned int batteryNumber ) const;
        
        //! Erase a battery
        void eraseBattery( unsigned int batteryNumber );
        
        //! Set charge in %
        void setPercentCharge(int percentCharge) { isEmpty_=false; percentCharge_ = percentCharge; };
        //! Access charget in %
        int percentCharge() const { return percentCharge_; };
        //! Set minutes to empty
        void setMinToEmpty(int minToEmpty) { isEmpty_=false; minToEmpty_ = minToEmpty; };
        //! Access minutes-to-empty
        int minToEmpty() const { return minToEmpty_; };
        //! Set message-to-system string
        void setMessageToSystem(const std::string &messageToSystem) { isEmpty_=false; messageToSystem_ = messageToSystem; };
        //! Access message-to-system string
        std::string messageToSystem() const { return messageToSystem_; };
        
        //! Access availableBatteries flags
        const std::vector<bool> &availableBatteries() const { return availableBatteries_; };
        //! Set availableBatteries flags
        std::vector<bool> &availableBatteries() { isEmpty_=false; return availableBatteries_; };
        //! Access chargingStates flags
        const std::vector<bool> &chargingStates() const { return chargingStates_; };
        //! Set chargingStates flags
        std::vector<bool> &chargingStates() { isEmpty_=false; return chargingStates_; };
        //! Access supplyingPowerStates flags
        const std::vector<bool> &supplyingPowerStates() const { return supplyingPowerStates_; };
        //! Set supplyingPowerStates flags
        std::vector<bool> &supplyingPowerStates() { isEmpty_=false; return supplyingPowerStates_; };
        //! Access chargePowerPresentStates flags
        const std::vector<bool> &chargePowerPresentStates() const { return chargePowerPresentStates_; };
        //! Set chargePowerPresentStates flags
        std::vector<bool> &chargePowerPresentStates() { isEmpty_=false; return chargePowerPresentStates_; };
        //! Access powerNoGoodStates flags
        const std::vector<bool> &powerNoGoodStates() const { return powerNoGoodStates_; };
        //! Set powerNoGoodStates flags
        std::vector<bool> &powerNoGoodStates() { isEmpty_=false; return powerNoGoodStates_; };
        //! Access chargeInhibitedStates flags
        const std::vector<bool> &chargeInhibitedStates() const { return chargeInhibitedStates_; }
        //! Set chargeInhibitedStates flags
        std::vector<bool> &chargeInhibitedStates() { isEmpty_=false; return chargeInhibitedStates_; }
        
        //! Access the latest raw record, useful for debugging
        const std::vector<std::string> &rawRecord() const { return rawRecord_; };
        //! Set the latest raw record
        std::vector<std::string> &rawRecord() { isEmpty_=false; return rawRecord_; };
 
    private:

        bool isEmpty_;
    
        // Average battery values
        int percentCharge_;
        int minToEmpty_;
        std::string messageToSystem_;

        // Battery module states. Each vector is always of size 8 because OceanServer's Battery
        // Management Modules have a maximum of 8 slots (either 2, 4, or 8 dependent on the model)
        std::vector<bool> availableBatteries_;
        std::vector<bool> chargingStates_;
        std::vector<bool> supplyingPowerStates_;
        std::vector<bool> chargePowerPresentStates_;
        std::vector<bool> powerNoGoodStates_;
        std::vector<bool> chargeInhibitedStates_;

        // the latest raw record, useful for debugging
        std::vector<std::string> rawRecord_;
        
        // key: slot number, data: a single smart battery module
        std::map<int,SmartBattery> batteries_;
};

//! Puts OceanServerSystem data into a human-readable string
std::string toString( const OceanServerSystem &system );

//! Puts OceanServerSystem data into a machine-readable ASCII string
std::string toLogString( const OceanServerSystem &system );
    
//! Updates all fields in 'to' with data from 'from'. Also reaps batteries in 'to' if they are not in 'from'.
//! Has persistence capabilities: if fields in 'from' are not set and corresponding fields in 'to' are set, the ones in 'to' are kept.
//! There's one exception: the rawRecord field is always updated
//! Use case: a class stores 'to' as a member variable, receives the latest records into 'from', calls this function to update 'to'.
//! The reaping capability makes sure that battery modules which are no longer connected don't persist.
void updateWithNewData( const OceanServerSystem &from, 
                        OceanServerSystem       &to );

//! Returns true if the charge power is present for at least one of the battery modules, otherwise false
bool isChargePowerPresent( const OceanServerSystem &batterySystem );
                        
} // namespace

#endif
