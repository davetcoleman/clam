/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2008-2010 Geoffrey Biggs
 *
 * hokuyo_aist Hokuyo URG laser scanner driver.
 *
 * This distribution is licensed to you under the terms described in the
 * LICENSE file included in this distribution.
 *
 * This work is a product of the National Institute of Advanced Industrial
 * Science and Technology, Japan. Registration number: H22PRO-1086.
 *
 * This file is part of hokuyo_aist.
 *
 * This software is licensed under the Eclipse Public License -v 1.0 (EPL). See
 * http://www.opensource.org/licenses/eclipse-1.0.txt
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include <hokuyo_aist/hokuyo_aist.h>
#include <hokuyo_aist/hokuyo_errors.h>

int main(int argc, char **argv)
{
    std::string port_options="type=serial,device=/dev/ttyACM0,timeout=1";
    bool verbose=false;

#if defined (WIN32)
    port_options = "type=serial,device=COM3,timeout=1";
#else
    int opt;
    // Get some options from the command line
    while((opt = getopt(argc, argv, "b:c:e:f:il:m:no:s:vh")) != -1)
    {
        switch(opt)
        {
            case 'o':
                port_options = optarg;
                break;
            case 'v':
                verbose = true;
                break;
            case '?':
            case 'h':
            default:
                std::cout << "Usage: " << argv[0] << " [options]\n\n";
                std::cout <<
                    "-o options\tPort options (see flexiport library).\n";
                std::cout <<
                    "-v\t\tPut the hokuyo_aist library into verbose mode.\n";
                return 1;
        }
    }
#endif // defined (WIN32)

    try
    {
        hokuyo_aist::Sensor laser; // Laser scanner object
        // Set the laser to verbose mode (so we see more information in the
        // console)
        if(verbose)
            laser.set_verbose(true);

        // Open the laser
        laser.open(port_options);
        // Turn the laser on
        laser.set_power(true);

        // Get some laser info
        hokuyo_aist::SensorInfo info;
        laser.get_sensor_info(info);
        std::cout << info.serial << '\n';

        // Close the laser
        laser.close();
    }
    catch(hokuyo_aist::BaseError &e)
    {
        std::cerr << "Caught exception: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

