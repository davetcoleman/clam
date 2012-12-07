#include <iostream>
#include <sstream>
#include <gbxutilacfr/trivialtracer.h>
#include <gbxsmartbatteryacfr/gbxsmartbatteryacfr.h>

using namespace std;

string toString( const vector<string> &stringList )
{
    stringstream ss;
    for (unsigned int i=0; i<stringList.size(); ++i)
    {
        ss << stringList[i];
    }
    return ss.str();
    
}

string checkMinToEmpty( const gbxsmartbatteryacfr::OceanServerSystem &data )
{
    if ( data.minToEmpty()<1 )
    {
        stringstream ss;
        ss << "YES: MinutesToEmpty is close to zero: " << data.minToEmpty();
        return ss.str();
    }
    else
        return "NO";
}

int main( int argc, char **argv )
{
    int opt;
    std::string port = "/dev/ttyS0";
    int debug = 4;
    
    // Get some options from the command line
    while ((opt = getopt(argc, argv, "p:v")) != -1)
    {
        switch ( opt )
        {
            case 'p':
                port = optarg;
                break;
            case 'v':
                debug = 5;
                break;
            default:
                cout << "Usage: " << argv[0] << " [-p port] [-v(erbose)]" << endl
                     << "-p port\tPort the oceanserver battery system is connected to. E.g. /dev/ttyS0" << endl;
                return 1;
        }
    }
    
    int numRecords = 0;
    gbxutilacfr::TrivialTracer tracer( debug );    
    
    try 
    {
        gbxsmartbatteryacfr::BatteryHealthWarningConfig config;
        config.expectedNumBatteries = 1;
        config.numCyclesThreshhold = 300;
        config.chargeTempThreshhold = 40.0;
        config.dischargeTempThreshhold = 45.0;
        config.chargeWarnThreshhold = 10;
        config.chargeDeviationThreshold = 10;
        
        gbxsmartbatteryacfr::OceanServer oceanserver( port, tracer );
        oceanserver.start();

        while(true)
        {
            cout << "Waiting for first record to arrive" << endl;
            if (oceanserver.haveData())
                break;
            sleep(1);
        }
        cout << "OK. We have some data. Starting to read now..." << endl;
        
        // run forever
        while (true)    
        {   
            cout << "===================" << endl
                 << "Reading record " << numRecords << endl;
            numRecords++;         
            
            gbxsmartbatteryacfr::OceanServerSystem data;
            oceanserver.getData(data);

//             cout << gbxsmartbatteryacfr::toString( data ) << endl
//                  << "minToEmpty<1?: " << checkMinToEmpty(data) << endl
//                  << "isChargePowerPresent?: " << isChargePowerPresent(data) << endl
//                  << "raw record: " << endl
//                  << toString(data.rawRecord()) << endl;
            
            vector<string> shortWarning;
            vector<string> verboseWarning;
            const bool printRawRecord = false;
            bool haveWarnings = conductAllHealthChecks( data, config, shortWarning, verboseWarning, printRawRecord );
            
            if (haveWarnings)
            {
                cout << "Short warning messages: " << endl;
                cout << toString(shortWarning) << endl << endl;
                cout << "Verbose warning messages:" << endl;
                cout << toString(verboseWarning) << endl;
            }
            else
            {
                cout << "All systems normal." << endl;
            }

            sleep(1);
        }
    }
    catch ( gbxsmartbatteryacfr::HardwareReadingException &e )
    {
        cout << "ERROR(test): Caught a hardware reading exception: " 
                << e.what() << endl 
                << "This shouldn't happen!" << endl;
        return 1;
    }
    catch ( gbxutilacfr::Exception &e )
    {
        cout << "ERROR(test): Caught a gbxutilacfr::Exception: " 
             << e.what() << endl 
             << "This shouldn't happen!" << endl;
        return 1;
    }
    catch ( std::exception &e )
    {
        cout << "ERROR(test): Caught an unknown exception: " 
             << e.what() << endl
             << "This shouldn't happen!" << endl;
        return 1;
    }
       
    return 0;
}

