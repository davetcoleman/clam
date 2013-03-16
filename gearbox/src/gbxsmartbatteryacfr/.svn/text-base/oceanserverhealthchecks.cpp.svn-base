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
#include "oceanserverhealthchecks.h"


using namespace std;

namespace gbxsmartbatteryacfr 
{
    namespace 
    {

        bool isRecordEmpty( const OceanServerSystem &batteryData,
                            vector<string>          &warnShort,
                            vector<string>          &warnVerbose)
        {
            if ( !batteryData.isEmpty() )
                return false;

            warnVerbose.push_back("The OceanServerSystem data record was empty");
            warnShort.push_back("EMPTY RECORD");
            return true;
        }

        string toString( const vector<string> &stringList )
        {
            stringstream ss;
            for (unsigned int i=0; i<stringList.size(); ++i)
            {
                ss << stringList[i];
            }
            return ss.str();

        }
        
    }
    
bool checkNumberOfBatteries( const OceanServerSystem  &batteryData, 
                             std::vector<std::string> &warnShort, 
                             std::vector<std::string> &warnVerbose,
                             int                       expectedNumBatteries )
{
    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;
    
    bool haveWarning = false;
    
    int numBatteries = 0;
    const vector<bool> &bats = batteryData.availableBatteries();
    
    for (unsigned int i=0; i<bats.size(); i++)
    {
        if (bats[i] == true) {
            numBatteries++;
        }
    }
    if ( numBatteries!=expectedNumBatteries )
    {
        stringstream ssWarnShort;
        ssWarnShort << "HAVE " << numBatteries << ", EXPECT " << expectedNumBatteries << " BAT! ";
        warnShort.push_back(ssWarnShort.str());
        stringstream ssWarnVerbose;
        ssWarnVerbose << "Found " << numBatteries << " battery modules (expected to see " << expectedNumBatteries << ")" << endl;
        warnVerbose.push_back( ssWarnVerbose.str() );
        haveWarning = true;
    }
    
    return haveWarning;
}

bool checkNumCycles( const OceanServerSystem &batteryData, 
                     vector<string>          &warnShort, 
                     vector<string>          &warnVerbose,
                     int                      numCyclesThreshhold,
                     bool                     printRawRecord  )
{
    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;

    bool haveWarning = false;
    
    map<int,SmartBattery>::const_iterator it;
    for (it=batteryData.batteries().begin(); it!=batteryData.batteries().end(); it++)
    {
        const SmartBattery &bat = batteryData.battery( it->first );
        if ( !bat.has(CycleCount) ) continue;
    
        int numCycles = bat.cycleCount();
        
        if ( numCycles > numCyclesThreshhold )
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << it->first << "): HIGH CYCLES! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "High charge cycles! Battery no " << it->first << " has had " << numCycles << " cycles (consider swapping at " << numCyclesThreshhold << " cycles)" << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }    
    }
    
    if (haveWarning && printRawRecord)
    {
        stringstream ssWarnVerbose;
        ssWarnVerbose << "Latest raw record: " << endl << toString(batteryData.rawRecord()) << endl;
        warnVerbose.push_back(ssWarnVerbose.str());
    }
    
    return haveWarning;
       
}

bool checkTemperatures(const OceanServerSystem &batteryData, 
                        vector<string>         &warnShort, 
                        vector<string>         &warnVerbose,
                        double                  chargeTempThreshhold,
                        double                  dischargeTempThreshhold,
                        bool                    printRawRecord   )
{
    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;

    bool haveWarning = false;
    
    map<int,SmartBattery>::const_iterator it;

    for (it=batteryData.batteries().begin(); it!=batteryData.batteries().end(); it++)
    {
        int batteryNumber = it->first;
        const SmartBattery &bat = batteryData.battery( batteryNumber );
        if ( !bat.has(Temperature) ) continue;

        assert( (int)batteryData.chargingStates().size() >= batteryNumber-1 );
        bool isCharging = batteryData.chargingStates()[batteryNumber-1];
    
        double tempThreshhold = 0.0;
        if (isCharging) {
            tempThreshhold = chargeTempThreshhold;
        } else {
            tempThreshhold = dischargeTempThreshhold;
        }
                    
        double temperature = bat.temperature();
        if ( temperature > tempThreshhold)
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << batteryNumber << "): HOT! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "High temperature! Battery no " << batteryNumber << " has " << temperature << "degC (threshhold: " << tempThreshhold << "degC)" << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }
    }
    
    if (haveWarning && printRawRecord)
    {
        stringstream ssWarnVerbose;
        ssWarnVerbose << "Latest raw record: " << endl << toString(batteryData.rawRecord()) << endl;
        warnVerbose.push_back(ssWarnVerbose.str());
    }
    
    return haveWarning;
}

bool checkCharges(const OceanServerSystem &batteryData, 
                  vector<string>          &warnShort, 
                  vector<string>          &warnVerbose,
                  int                      chargeWarnThreshhold,
                  int                      chargeDeviationThreshold )
{

    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;

    // if the system is charging, don't bother issuing warnings
    if ( isChargePowerPresent(batteryData) ) return false;
    
    bool haveWarning = false;
    
    map<int,SmartBattery>::const_iterator it;
    
    for (it=batteryData.batteries().begin(); it!=batteryData.batteries().end(); it++)
    {
        int batteryNumber = it->first;
        const SmartBattery &bat = batteryData.battery( batteryNumber );
        if ( !bat.has(RelativeStateOfCharge) ) continue;
    
        int charge = bat.relativeStateOfCharge();
        const int avgCharge = batteryData.percentCharge();
        
        // check whether battery charge is lower than the average
        if ( charge < (avgCharge - chargeDeviationThreshold) )
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << batteryNumber << "): INCONSISTENT CHARGE! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "Inconsistent charge! Battery no " << batteryNumber << "'s charge is " << charge << "% (average: " << avgCharge << "%)"  << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }

        if (charge < chargeWarnThreshhold)
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << batteryNumber << "): LOW CHARGE! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "Low charge! Battery no " << batteryNumber << "'s charge is " << charge << "% (threshhold: " << chargeWarnThreshhold << "%)" << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }  
    }      
    
    return haveWarning;
}

bool checkModuleHealth( const OceanServerSystem &batteryData,
                        vector<string>          &warnShort, 
                        vector<string>          &warnVerbose )
{

    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;

    bool haveWarning = false;
    
    // check the flags
    const vector<bool> &badPower = batteryData.powerNoGoodStates();
    for (unsigned int i=0; i<badPower.size(); i++)
    {
        if (badPower[i]==true)
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << i+1 << "): BAD POWER! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "Power-no-good flag of battery number " << i+1 << " is set!" << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }
    }
    
    const vector<bool> &chargeInhibit = batteryData.chargeInhibitedStates();
    for (unsigned int i=0; i<chargeInhibit.size(); i++)
    {
        if (chargeInhibit[i]==true)
        {
            haveWarning = true;
            stringstream ssWarnShort;
            ssWarnShort << "BAT(" << i+1 << "): CHARGE INHIBITED! ";
            warnShort.push_back(ssWarnShort.str());
            stringstream ssWarnVerbose;
            ssWarnVerbose << "Charge-inhibited flag of battery number " << i+1 << " is set!" << endl;
            warnVerbose.push_back(ssWarnVerbose.str());
        }
    }
    
    return haveWarning;
}

bool conductAllHealthChecks( const OceanServerSystem          &batteryData,
                             const BatteryHealthWarningConfig &batteryConfig,
                             std::vector<std::string>         &warnShort,
                             std::vector<std::string>         &warnVerbose,
                             bool                              printRawRecord   )
{
    if ( isRecordEmpty(batteryData, warnShort, warnVerbose) ) return true;

    bool warnNumBatteries = checkNumberOfBatteries( batteryData, warnShort, warnVerbose, batteryConfig.expectedNumBatteries );
    bool warnModule = checkModuleHealth( batteryData, warnShort, warnVerbose );
    bool warnCycle = checkNumCycles( batteryData, warnShort, warnVerbose, batteryConfig.numCyclesThreshhold, false );
    bool warnTemp = checkTemperatures( batteryData, warnShort, warnVerbose, batteryConfig.chargeTempThreshhold, batteryConfig.dischargeTempThreshhold, false );
    bool warnCharge = checkCharges( batteryData, warnShort, warnVerbose, batteryConfig.chargeWarnThreshhold, batteryConfig.chargeDeviationThreshold );

    bool haveWarnings = warnNumBatteries || warnModule || warnCycle || warnTemp || warnCharge;
    
    if (printRawRecord && haveWarnings )
    {
        stringstream ssWarnVerbose;
        ssWarnVerbose << "Latest raw record: " << endl << toString(batteryData.rawRecord()) << endl;
        warnVerbose.push_back(ssWarnVerbose.str());
    }
    
    return haveWarnings;
}

    
    
}

