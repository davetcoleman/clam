/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <gbxgarminacfr/driver.h>
#include <gbxutilacfr/trivialtracer.h>
#include <gbxutilacfr/trivialstatus.h>
#include <gbxutilacfr/mathdefs.h>

using namespace std;

//
// Instantiates the driver, reads a messages
//
int main( int argc, char **argv )
{
    int opt;
    // defaults
    string port = "/dev/ttyS0";
    bool quiet = false;
    int numReads = 30;
    int debugLevel = 0;

    // Get some options from the command line
    while ((opt = getopt(argc, argv, "p:n:d:q")) != -1)
    {
        switch ( opt )
        {
        case 'p':
            port = optarg;
            break;
        case 'n':
            numReads=atoi(optarg);
            break;
        case 'd':
            debugLevel=atoi(optarg);
            break;
        case 'q':
            quiet = true;
            break;
        default:
            cout << "Usage: " << argv[0] << " [-p port]" << endl << endl
                 << "-p port    \tPort the device is connected to. E.g. /dev/ttyS0" << endl
                 << "-n numReads\tNumber of times to read from the device" << endl
                 << "-q         \tMakes normal operation quiet, only errors are traced." << endl;
            return 1;
        }
    }

    // Set up the device configuration
    gbxgarminacfr::Config config;
    config.device        = port;
    config.protocol      = "Garmin";
    config.readGga       = true;
    config.readVtg       = true;
    config.readRme       = true;
    config.readRmc       = false;
    config.ignoreUnknown = false;
    cout << "Using configuration: " << config.toString() << endl;

    // Instantiate objects to handle messages from the driver
    if ( quiet )
        debugLevel = 0;
    gbxutilacfr::TrivialTracer tracer( debugLevel );
    gbxutilacfr::TrivialStatus status( tracer );

    // Instantiate the driver itself
    std::auto_ptr<gbxgarminacfr::Driver> device;
    try 
    {
        device.reset( new gbxgarminacfr::Driver( config, tracer, status, debugLevel ) );
    }
    catch ( const std::exception& e )
    {
        cout <<"Test: Failed to init device: "<<e.what() << endl;
        return 1;
    }

    // Create data structure to store sensor data
    std::auto_ptr<gbxgarminacfr::GenericData> data;

    // Read a few times
    for ( int i=0; i < numReads; i++ )
    {
        try 
        {
            data = device->read();
            
            // find out which data type it is
            switch ( data->type() )
            {
                case gbxgarminacfr::GpGga :
                {
                    gbxgarminacfr::GgaData* d = (gbxgarminacfr::GgaData*)data.get();
                    if ( !quiet ) {
                        cout<<"GPGGA:"<<endl
                            <<"  satellites     = "<<d->satellites<<endl
                            <<"  fix type       = "<<d->fixType<<endl
                            <<"  longitude      = "<<d->longitude<<endl
                            <<"  latitude       = "<<d->latitude<<endl;
                    }
                    break;
                }
                case gbxgarminacfr::GpVtg :
                {
                    gbxgarminacfr::VtgData* d = (gbxgarminacfr::VtgData*)data.get();
                    if ( !quiet ) {
                        cout<<"GPVTG:"<<endl
                            <<"   speed         = "<<d->speed<<endl;
                    }
                    break;
                }
                case gbxgarminacfr::PgRme :
                {
                    gbxgarminacfr::RmeData* d = (gbxgarminacfr::RmeData*)data.get();
                    if ( !quiet ) {
                        cout<<"PGRME:"<<endl
                            <<"   horiz error   = "<<d->horizontalPositionError<<endl;
                    }
                    break;
                }
                default :
                    cout<<"Unknown message"<<endl;
            }

            // no need to delete the data object, the auto pointer will delete it automatically

            if ( !quiet )
                cout<<"Test: Got data "<<i+1<<" of "<<numReads<<endl;
        }
        catch ( const std::exception& e )
        {
            cout <<"Test: Failed to read data: "<<e.what()<<endl;
        }    
    }
    return 0;
}
