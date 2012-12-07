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

#include <stdio.h>
#include <math.h>
#include <iostream>

#include <hokuyo_aist/hokuyo_aist.h>
#include <hokuyo_aist/hokuyo_errors.h>
#include <flexiport/flexiport.h>

int main(int argc, char **argv)
{
    std::string port_options("type=serial,device=/dev/ttyACM0,timeout=1");
    double start_angle(0.0), end_angle(0.0);
    int first_step(-1), last_step(-1);
    int multiecho_mode(0);
    unsigned int baud(19200), speed(0), cluster_count(1);
    bool get_intensities(false), get_new(false), verbose(false);

#if defined(WIN32)
    port_options = "type=serial,device=COM3,timeout=1";
#else
    int opt;
    // Get some options from the command line
    while((opt = getopt(argc, argv, "b:c:e:f:il:m:no:s:u:vh")) != -1)
    {
        switch(opt)
        {
            case 'b':
                sscanf(optarg, "%d", &baud);
                break;
            case 'c':
                sscanf(optarg, "%d", &cluster_count);
                break;
            case 'e':
                sscanf(optarg, "%lf", &end_angle);
                break;
            case 'f':
                sscanf(optarg, "%d", &first_step);
                break;
            case 'i':
                get_intensities = true;
                break;
            case 'l':
                sscanf(optarg, "%d", &last_step);
                break;
            case 'm':
                sscanf(optarg, "%d", &speed);
                break;
            case 'n':
                get_new = true;
                break;
            case 'o':
                port_options = optarg;
                break;
            case 's':
                sscanf(optarg, "%lf", &start_angle);
                break;
            case 'u':
                sscanf(optarg, "%d", &multiecho_mode);
                break;
            case 'v':
                verbose = true;
                break;
            case '?':
            case 'h':
            default:
                std::cout << "Usage: " << argv[0] << " [options]\n\n";
                std::cout << "-b baud\t\tBaud rate to set the laser to "
                    "*after* connecting.\n";
                std::cout << "-c count\tCluster count.\n";
                std::cout << "-e angle\tEnd angle to get ranges to.\n";
                std::cout << "-f step\t\tFirst step to get ranges from.\n";
                std::cout << "-i\t\tGet intensity data along with ranges.\n";
                std::cout << "-l step\t\tLast step to get ranges to.\n";
                std::cout << "-m speed\tMotor speed.\n";
                std::cout <<
                    "-n\t\tGet new ranges instead of latest ranges.\n";
                std::cout <<
                    "-o options\tPort options (see flexiport library).\n";
                std::cout << "-s angle\tStart angle to get ranges from.\n";
                std::cout << "-u mode\tMulti-echo detection:\n";
                std::cout << "\t\t0: Off (default), 1: Front, 2: Middle, "
                    "3: Rear, 4: Average\n";
                std::cout <<
                    "-v\t\tPut the hokuyo_aist library into verbose mode.\n";
                return 1;
        }
    }
#endif // defined(WIN32)

    try
    {
        hokuyo_aist::Sensor laser; // Laser scanner object
        // Set the laser to verbose mode (so we see more information in the
        // console)
        if(verbose)
            laser.set_verbose(true);

        // Open the laser
        laser.open(port_options);

        // Calibrate the laser time stamp
        std::cout << "Calibrating laser time\n";
        laser.calibrate_time();
        std::cout << "Calculated offset: " << laser.time_offset() << "ns\n";
        std::cout << "Calculated drift rate: " << laser.drift_rate() << '\n';
        std::cout << "Calculated skew alpha: " << laser.skew_alpha() << '\n';

        // Turn the laser on
        laser.set_power(true);
        // Set the baud rate
        try
        {
            laser.set_baud(baud);
        }
        catch(hokuyo_aist::BaudrateError &e)
        {
            std::cerr << "Failed to change baud rate: " << e.what() << '\n';
        }
        catch(hokuyo_aist::ResponseError &e)
        {
            std::cerr << "Failed to change baud rate: " << e.what() << '\n';
        }
        catch(...)
        {
            std::cerr << "Failed to change baud rate\n";
        }
        // Set the motor speed
        try
        {
            laser.set_motor_speed(speed);
        }
        catch(hokuyo_aist::MotorSpeedError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        catch(hokuyo_aist::ResponseError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        // Set multiecho mode
        switch(multiecho_mode)
        {
            case 1:
                laser.set_multiecho_mode(hokuyo_aist::ME_FRONT);
                break;
            case 2:
                laser.set_multiecho_mode(hokuyo_aist::ME_MIDDLE);
                break;
            case 3:
                laser.set_multiecho_mode(hokuyo_aist::ME_REAR);
                break;
            case 4:
                laser.set_multiecho_mode(hokuyo_aist::ME_AVERAGE);
                break;
            case 0:
            default:
                laser.set_multiecho_mode(hokuyo_aist::ME_OFF);
                break;
        }

        // Get some laser info
        std::cout << "Laser sensor information:\n";
        hokuyo_aist::SensorInfo info;
        laser.get_sensor_info(info);
        std::cout << info.as_string();

        // Get range data
        hokuyo_aist::ScanData data;
        if((first_step == -1 && last_step == -1) &&
            (start_angle == 0.0 && end_angle == 0.0))
        {
            // Get all ranges
            if(get_new)
                laser.get_new_ranges(data, -1, -1, cluster_count);
            else if(get_intensities)
                laser.get_new_ranges_intensities(data, -1, -1, cluster_count);
            else
                laser.get_ranges(data, -1, -1, cluster_count);
        }
        else if(first_step != -1 || last_step != -1)
        {
            // Get by step
            if(get_new)
                laser.get_new_ranges(data, first_step, last_step,
                        cluster_count);
            else if(get_intensities)
                laser.get_new_ranges_intensities(data, first_step, last_step,
                        cluster_count);
            else
                laser.get_ranges(data, first_step, last_step, cluster_count);
        }
        else
        {
            // Get by angle
            if(get_new)
                laser.get_new_ranges_by_angle(data, start_angle, end_angle,
                        cluster_count);
            else if(get_intensities)
                laser.get_new_ranges_intensities_by_angle(data, start_angle,
                        end_angle, cluster_count);
            else
                laser.get_ranges_by_angle(data, start_angle, end_angle,
                        cluster_count);
        }

        std::cout << "Measured data:\n";
        std::cout << data.as_string();

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

