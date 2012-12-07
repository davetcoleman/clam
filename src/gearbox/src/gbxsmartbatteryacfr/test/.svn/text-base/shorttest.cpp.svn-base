#include <iostream>
#include <gbxutilacfr/trivialtracer.h>
#include <gbxsmartbatteryacfr/gbxsmartbatteryacfr.h>

using namespace std;

int main( int argc, char **argv )
{
    int opt;
    std::string port = "/dev/ttyS0";
    int debug = 0;
    
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
    
    const unsigned int numRecords = 5;
    cout << "INFO(test): The plan is to read " << numRecords << " records from the oceanserver system and display the results." << endl << endl;

    gbxutilacfr::TrivialTracer tracer( debug );    
    
    try 
    {
        gbxsmartbatteryacfr::OceanServer *oceanServerThread = new gbxsmartbatteryacfr::OceanServer( port, tracer );
        gbxiceutilacfr::ThreadPtr oceanServerThreadPtr = oceanServerThread;
        oceanServerThread->start();

        while(true)
        {
            cout << "Waiting for first record to arrive" << endl;
            if (oceanServerThread->haveData())
                break;
            sleep(1);
        }
        cout << "OK. We have some data. Starting to read now..." << endl;

        
        for (unsigned int i=0; i<=numRecords; i++)
        {
            gbxsmartbatteryacfr::OceanServerSystem data;
            oceanServerThread->getData(data);
            
            cout << "TRACE(test): Reading record " << i << ": " << endl
                 << "=================================" << endl << endl
                 << gbxsmartbatteryacfr::toString( data ) << endl;

            sleep(1);
        }
        
        // stop the thread
        gbxiceutilacfr::stopAndJoin(oceanServerThreadPtr);
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

    cout << "INFO(test): Successfully read " << numRecords << " records." << endl;
    
    return 0;
}

