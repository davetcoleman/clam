/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBX_SMARTBATTERY_PARSING_H
#define GBX_SMARTBATTERY_PARSING_H

#include <stdint.h>

#include <vector>
#include <map>

#include <gbxutilacfr/tracer.h>

namespace gbxsmartbatteryacfr
{
    
//! Expects 2 hex characters and translates them into a vector of boolean flags
//! May throw ParsingException
void readFlags( const std::string &str, 
                std::vector<bool> &flags );

//! Expects 4 hex characters, returns temperature [degC]
//! May throw ParsingException
double readTemperature( const std::string &str );

//! Expects 4 hex characters, returns current [A]
//! May throw ParsingException
double readCurrent( const std::string &str );

//! Expects 4 hex characters, returns voltage [V]
//! May throw ParsingException
double readVoltage( const std::string &str );

//! Expects 2 hex characters, returns number of batteries
//! May throw ParsingException
int readNumBatteries( const std::string &str );

//! Expects 4 hex characters, returns percentage [%]
//! May throw ParsingException
int readPercentWord( const std::string &str );

//! Expects 2 hex characters, returns percentage [%]
//! May throw ParsingException
int readPercentByte( const std::string &str );

//! Expects 4 hex characters, returns minutes
//! May throw ParsingException
int readMinutes( const std::string &str );

//! Expects 4 hex characters, returns capacity [%]
//! May throw ParsingException
int readCapacity( const std::string &str );

//! Expects 4 hex characters, translates them into uint16_t
//! May throw ParsingException
uint16_t read16Flags( const std::string &str );

//! Expects 4 hex characters, returns a count
//! May throw ParsingException
int readCount( const std::string &str );

//! Expects 4 hex characters, returns a number
//! May throw ParsingException
int readNumber( const std::string &str );

//! Expects 4 hex characters, returns a rate
//! May throw ParsingException
int readRate( const std::string &str );

//! Computes an XOR checksum from 'input' (skips the first character).
//! Returns true if it matches with 'expected', otherwise false
bool isChecksumValid( const std::string &input, 
                      const std::string &expected );  
                     
//! Decomposes 'fields' (a flat list of keys and values) into 'pairs' 
//! (a map of keys and values). May throw ParsingException.
void toKeyValuePairs( const std::vector<std::string>    &fields,
                      std::map<std::string,std::string> &pairs,
                      gbxutilacfr::Tracer               &tracer );

}

#endif
