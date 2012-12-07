/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_OCEANSERVER_HEALTH_CHECKS
#define GBX_OCEANSERVER_HEALTH_CHECKS

#include <gbxsmartbatteryacfr/oceanserversystem.h>

namespace gbxsmartbatteryacfr
{

//! Issues a short and verbose warning if the number of recharge cycles of one or more batteries are exceeded
//! Optionally includes the raw battery record in warnVerbose
//! Returns true if there was a warning otherwise false
bool checkNumCycles( const OceanServerSystem  &batteryData, 
                     std::vector<std::string> &warnShort, 
                     std::vector<std::string> &warnVerbose,
                     int                       numCyclesThreshhold,
                     bool                      printRawRecord = false );

//! Issues a short and verbose warning if the temperature threshholds are of one or more batteries are exceeded
//! Optionally includes the raw battery record in warnVerbose
//! Returns true if there was a warning otherwise false
bool checkTemperatures(const OceanServerSystem  &batteryData, 
                       std::vector<std::string> &warnShort, 
                       std::vector<std::string> &warnVerbose,
                       double                    chargeTempThreshhold,
                       double                    dischargeTempThreshhold,
                       bool                      printRawRecord = false );
                       
//! Issues a short and verbose warning if the charge of one or more batteries is below the threshold or if the charge of any battery deviates by chargeDeviationThreshold percent from the average of all batteries
//! Returns true if there was a warning otherwise false
bool checkCharges(const OceanServerSystem  &batteryData, 
                  std::vector<std::string> &warnShort, 
                  std::vector<std::string> &warnVerbose,
                  int                       chargeWarnThreshhold,
                  int                       chargeDeviationThreshold );

//! Issues a short and verbose warning if any of the individual modules have "bad power" or are "charge-inhibited"
//! Returns true if there was a warning otherwise false
bool checkModuleHealth( const OceanServerSystem  &batteryData, 
                        std::vector<std::string> &warnShort, 
                        std::vector<std::string> &warnVerbose );

//! Issues a short and verbose warning if the number of expected batteries are not installed
//! Returns true if there was a warning otherwise false                        
bool checkNumberOfBatteries( const OceanServerSystem  &batteryData, 
                             std::vector<std::string> &warnShort, 
                             std::vector<std::string> &warnVerbose,
                             int                       expectedNumBatteries );

//! Config structure for battery health checks
//! The threshholds represent acceptable values before warnings are issued
struct BatteryHealthWarningConfig 
{
    //! Expected number of battery modules installed
    int expectedNumBatteries;
    //! Maximum number of battery recharge cycles before a warning is issued
    int numCyclesThreshhold;
    //! Maximum charging temperature before a warning is issued
    double chargeTempThreshhold;
    //! Maximum discharging temperature before a warning is issued
    double dischargeTempThreshhold;
    //! Minimum charge in percent before a warning is issued
    int chargeWarnThreshhold;
    //! Minimum deviation of a single battery from the average of all batteries before a warning is issued (in percent)
    int chargeDeviationThreshold;
};

//! Conducts all of the health checks above given a battery health configuration
//! Adds up short and verbose warning messages
//! Optionally includes the raw battery record at the end of warnVerbose if any check generated a warning
//! Returns true if there were warnings otherwise false
bool conductAllHealthChecks( const OceanServerSystem          &batteryData,
                             const BatteryHealthWarningConfig &batteryCheckConfig,
                             std::vector<std::string>         &warnShort,
                             std::vector<std::string>         &warnVerbose,
                             bool                              printRawRecord = false );
                       



} // namespace

#endif
