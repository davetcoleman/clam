/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include <iostream>
#include <sstream>
#include <memory>
#include <vector>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <limits.h>

#include <gbxutilacfr/trivialtracer.h>
#include <gbxutilacfr/exceptions.h>

#include <gbxnovatelacfr/driver.h>

using namespace std;
namespace gna = gbxnovatelacfr;

void usage(char *progName, const char *optString){
    printf("Usage: %s -m [mode] -p [port] -b [baud] -i [imutype] -t [time to run] -d -v -h\n", progName);
    printf("\t[mode]:\t\t\"gps\" to set the system up in GPS only mode\n"
            "\t\t\t\"ins\" to run INS/GPS mode (default)\n");
    printf("\t[port]:\t\tSerial port the receiver is connecteed to (default \"/dev/ttyS0\")\n");
    printf("\t[baud]:\t\tBaud rate to set up the serial port (9600, 19200, 38400, 115200, 230400), default is 115200\n");
    printf("\t[imutype]:\ttype of imu you are using (default \"IMU_HG1700_AG62\")\n");
    printf("\t[time to run]:\trun for so many seconds after we get data the first time (default 10)\n");
    printf("\t-d:\t\tshow debug info (default is off)\n");
    printf("\t-v:\t\tbe verbose, actually print out the data (default is off)\n");
    printf("\t-h:\t\tprint this message\n");
    printf("Options to decode: \t\"%s\"\n", optString);
    return;
}

int main(int argc, char *argv[]){
    int opt;
    const char *optString = ":m:p:b:i:t:dvh";

    //use sensible defaults
    string mode = "ins";
    string port = "/dev/ttyS0";
    int baud = 115200;
    double runTime = 10.0;
    string imuType = "IMU_HG1700_AG62";
    bool showDebug = false;
    bool verbose = false;

    opterr = 0; // we do our own error reporting
    while ((opt = getopt (argc, argv, optString)) != -1)
    {
        switch (opt)
        {
            case 'm':
                mode = optarg;
                break;
            case 'p':
                port = optarg;
                break;
            case 'b':
                baud = atoi( optarg );
                break;
            case 'i':
                imuType = optarg;
                break;
            case 't':
                runTime = atof( optarg );
                break;
            case 'd':
                showDebug = true;
                break;
            case 'v':
                verbose = true;
                break;
            case 'h':
                usage(argv[0], optString);
                return EXIT_SUCCESS;
            case '?':
                printf("Unknown option character: \"%c\"\n\n", optopt);
                usage(argv[0], optString);
                return EXIT_FAILURE;
            case ':':
                printf("Missing option-argument to \"-%c\"\n\n", optopt);
                usage(argv[0], optString);
                return EXIT_FAILURE;
            default:
                printf("?? Can't happen: getopt returned character code 0%o ??\n\n", opt);
                usage(argv[0], optString);
                return EXIT_FAILURE;
        }
    }
    cout << "\n-------------------------------------------------\n";
    cout << "| Starting with:\n";
    cout << "|\tmode:\t\t" << mode << "\n";
    cout << "|\tport:\t\t" << port << "\n";
    cout << "|\tbaud:\t\t" << baud << "\t\t[bps]\n";
    cout << "|\truntime:\t" << runTime << "\t\t[sec]\n";
    cout << "|\tImu:\t\t" << imuType << "\n";
    cout << "|\tverbose:\t" << verbose << "\n";
    cout << "|\tshow debug:\t" << showDebug << "\n";
    cout << "-------------------------------------------------\n";

    //read data from the driver for ~ [runTime] sec
    int maxGpsPosCnt;
    int maxGpsVelCnt;
    int maxInsPvaCnt;
    int maxRawImuCnt;
    int maxFaultCnt = 2;
    // bit dodgy to do this with counters, at least make sure we don't overflow
    if(20.0*runTime < INT_MAX){
        maxGpsPosCnt = 20*runTime;
        maxGpsVelCnt = 20*runTime;
    }
    else{
        maxGpsPosCnt = INT_MAX;
        maxGpsVelCnt = INT_MAX;
        cout << "!runtime too high, reset to " << INT_MAX/20 << "sec!\n";
    }
    if(100.0*runTime < INT_MAX){
        maxInsPvaCnt = 100*runTime;
        maxRawImuCnt = 100*runTime;
    }
    else{
        maxInsPvaCnt = INT_MAX;
        maxRawImuCnt = INT_MAX;
        cout << "!runtime too high, reset to " << INT_MAX/100 << "sec!\n";
    }
    cout << "\n";

    /**************************************************
     * This is roughly what user-code should look like
     **************************************************/

    //create a config (the easy way)
    auto_ptr<gna::Config > cfg;
    auto_ptr<gna::SimpleConfig > simpleCfg;
    auto_ptr<gna::GpsOnlyConfig > gpsOnlyCfg;
    if(0 == mode.compare("ins")){
        vector<double > imuToGpsOffset(3,0.0);  // made up; this MUST be correct for real work (see gbxnovatelacfr::Config::imuToGpsOffset_)
        simpleCfg.reset( new gna::SimpleConfig(port, baud, imuType, imuToGpsOffset) );
        cfg.reset( new gna::Config(*simpleCfg.get()) );
        assert(0 != cfg.get());
        // set up a couple of extra parameters
        cfg->enableInsPhaseUpdate_ = true;
        cfg->enableRTK_ = true;
    }
    else if(0 == mode.compare("gps")){
        gpsOnlyCfg.reset( new gna::GpsOnlyConfig(port, baud) );
        cfg.reset( new gna::Config(*gpsOnlyCfg.get()) );
        assert(0 != cfg.get());
        // set up a couple of extra parameters
        // some hardware combinations may not support this
        // cfg->enableSBAS_ = true;
        // cfg->enableCDGPS_ = true;
    }
    else{
        cout << "invalid mode: " <<  mode << "\n";
        return EXIT_FAILURE;
    }
    //check that it makes sense
    if(!cfg->isValid()){
        cout << "Invalid configuration!\n";
        cout << cfg->toString();
        return EXIT_FAILURE;
    }

    //create the driver; this gets things rolling (opens the serial port, sets up the device)
    auto_ptr<gna::Driver > driver;
    int errorCnt = 0;
    do{
        try{
            driver.reset( new gna::Driver( *(cfg.get()) ) );
        }
        catch(std::exception &e){
            cout << e.what() << "\nWaiting 5 sec before re-trying.\n\n";
            sleep(5);
        }
        catch(...){
            cout << "caught unknown exception!\nWaiting 5 sec before re-trying.\n\n";
            sleep(5);
        }
    }while(0 == driver.get() && errorCnt++<5); // try up to 5 times 
    //check that we succeeded
    if(0 == driver.get()){
        cout << "failed to set up driver!\n";
        return EXIT_FAILURE;
    }

    while(0 < maxRawImuCnt
            && 0 < maxInsPvaCnt
            && 0 < maxGpsVelCnt
            && 0 < maxGpsPosCnt
            && 0 < maxFaultCnt){
        auto_ptr<gna::GenericData > generic;
        try{
            generic = driver->read();
        }
        catch( const std::exception& e ){
            cout << e.what() << "\n";
            maxFaultCnt--;
            continue;
        }
        //figure out what we've got
        switch(generic->type()){
            case gna::InsPva:
                {
                    gna::InsPvaData *data = dynamic_cast<gna::InsPvaData *>(generic.get());
                    assert(0 != data);
                    //process data directly, or stuff it into your own container and pass it on
                    //!beware: data will get deleted when generic goes out of scope (next iteration of the loop in this case)!
                    if(verbose){
                        cout << data->toString() << "\n";
                    }else{
                        cout << "got inspva\n";
                    }
                    maxInsPvaCnt--;
                }
                break;
            case gna::BestGpsPos:
                {
                    gna::BestGpsPosData *data = dynamic_cast<gna::BestGpsPosData *>(generic.get());
                    assert(0 != data);
                    //process data directly, or stuff it into your own container and pass it on
                    //!beware: data will get deleted when generic goes out of scope (next iteration of the loop in this case)!
                    if(verbose){
                        cout << data->toString() << "\n";
                    }else{
                        cout << "got bestgpspos\n";
                    }
                    maxGpsPosCnt--;
                }
                break;
            case gna::BestGpsVel:
                {
                    gna::BestGpsVelData *data = dynamic_cast<gna::BestGpsVelData *>(generic.get());
                    assert(0 != data);
                    //process data directly, or stuff it into your own container and pass it on
                    //!beware: data will get deleted when generic goes out of scope (next iteration of the loop in this case)!
                    if(verbose){
                        cout << data->toString() << "\n";
                    }else{
                        cout << "got bestgpsvel\n";
                    }
                    maxGpsVelCnt--;
                }
                break;
            case gna::RawImu:
                {
                    gna::RawImuData *data = dynamic_cast<gna::RawImuData *>(generic.get());
                    assert(0 != data);
                    //process data directly, or stuff it into your own container and pass it on
                    //!beware: data will get deleted when generic goes out of scope (next iteration of the loop in this case)!
                    if(verbose){
                        cout << data->toString() << "\n";
                    }else{
                        cout << "got rawimu\n";
                    }
                    maxRawImuCnt--;
                }
                break;
            default:
                cout << "Got unknown message!\n";
                maxFaultCnt--;
                break;
        }
    }
    if(maxFaultCnt > 0){
        return EXIT_SUCCESS;
    }else{
        return EXIT_FAILURE;
    }
}
