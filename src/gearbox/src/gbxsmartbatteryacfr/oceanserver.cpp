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
#include <gbxsmartbatteryacfr/exceptions.h>
#include "oceanserver.h"

namespace gbxsmartbatteryacfr {

static const int MAX_EXCEPTIONS_BEFORE_RESET = 10;
static const int MAX_EXCEPTIONS_BEFORE_CRITICAL = 20;

OceanServer::OceanServer( const std::string   &port, 
                          gbxutilacfr::Tracer &tracer)
    : gbxiceutilacfr::SafeThread( tracer ),
      tracer_(tracer),
      exceptionCounter_(0),
      exceptionString_("")
{
    reader_.reset(new gbxsmartbatteryacfr::OceanServerReader( port, tracer_ ));
}

void
OceanServer::walk()
{
    while ( !isStopping() )
    {

        gbxsmartbatteryacfr::OceanServerSystem latestData;
        gbxsmartbatteryacfr::OceanServerSystem mergedData;

        try
        {
            // read new data, this may throw
            reader_->read(latestData);
            
            // successful read: reset counter and string
            exceptionCounter_ = 0;
            exceptionString_ = "";

            // merge latest data into a full record
            if ( !dataStore_.isEmpty() )
                dataStore_.get( mergedData );
            gbxsmartbatteryacfr::updateWithNewData( latestData, mergedData );
            dataStore_.set( mergedData );
        }
        catch ( gbxsmartbatteryacfr::ParsingException &e )
        {
            stringstream ss;
            ss << "OceanServer: " << __func__ << ": Caught a parsing exception: "
            << e.what() << endl
            << "This can happen sometimes. Will continue regardless.";
            tracer_.debug( ss.str(), 3 );
            
            exceptionCounter_++;
            stringstream ssEx;
            ssEx << e.what() << endl;
            for (unsigned int i=0; i<latestData.rawRecord().size(); i++)
                ssEx << latestData.rawRecord()[i] << endl;
            ssEx << endl;
            exceptionString_ = exceptionString_ + ssEx.str();
            
            if (exceptionCounter_ >= MAX_EXCEPTIONS_BEFORE_RESET)
            {
                stringstream ss;
                ss << "OceanServer: " << __func__ << ": Caught " << MAX_EXCEPTIONS_BEFORE_RESET
                << " ParsingExceptions in a row. Resetting the reader now...";
                tracer_.warning( ss.str() );
                reader_->reset();
            }
            
            if (exceptionCounter_ >= MAX_EXCEPTIONS_BEFORE_CRITICAL)
            {
                ss.str("");
                ss << "OceanServer: " << __func__ << ": Caught " << MAX_EXCEPTIONS_BEFORE_CRITICAL
                << " ParsingExceptions in a row. Something must be wrong. Here's the history: " << endl
                << exceptionString_;
                throw gbxutilacfr::Exception( ERROR_INFO, ss.str() );
            }
        }

    }
}

void 
OceanServer::getData(gbxsmartbatteryacfr::OceanServerSystem &data)
{
    if ( dataStore_.isEmpty() ) return;
    dataStore_.get( data );
}

bool
OceanServer::haveData()
{
    return ( !dataStore_.isEmpty() );
}

}
