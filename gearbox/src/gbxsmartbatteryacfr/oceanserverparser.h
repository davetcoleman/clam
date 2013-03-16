/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_OCEANSERVER_PARSER_H
#define GBX_OCEANSERVER_PARSER_H

#include <map>
#include <gbxutilacfr/tracer.h>
#include <gbxsmartbatteryacfr/oceanserversystem.h>

namespace gbxsmartbatteryacfr
{
    
//! 
//! Class to parse the hex data the oceanserver battery controller spits out
//!
//! @author Tobias Kaupp
//!
class OceanServerParser
{
public:
    
    OceanServerParser( gbxutilacfr::Tracer &tracer );
    
    //! Expects a full record of batterydata as a stringList (one line per string) produced by the oceanserver controller.
    //! Parses each line and sets corresponding fields in batterySystem
    void parse( std::vector<std::string> &stringList, 
                OceanServerSystem        &batterySystem );
    
    //! Checks whether the passed string (one line) is the first line of the record
    bool atBeginningOfRecord( const std::string &line );

private:       
    
    gbxutilacfr::Tracer &tracer_;
            
    // parsing functions
    void parseFields( std::vector<std::string>       &fields, 
                      OceanServerSystem              &batterySystem );
    
    void parseSystemData( const std::map<std::string,std::string> &keyValuePairs,
                          OceanServerSystem                       &batterySystem);
    
    void parseControllerData( const std::map<std::string,std::string> &keyValuePairs,
                              OceanServerSystem                       &batterySystem);
    
    void parseSingleBatteryData( const std::map<std::string,std::string> &keyValuePairs, 
                                 unsigned int                             batteryNum,
                                 OceanServerSystem                       &batterySystem);
    
};


} // namespace

#endif
