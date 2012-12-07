#include <iostream>
#include <gbxserialacfr/serial.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip> 

using namespace std;

//
// A simple utility program for testing serial ports in loopback.
//   Put the port in loopback (ie connect pins 2 & 3), then type stuff.
//   It should be echoed back on the terminal.
//

std::vector<std::string>
getStringList()
{
    std::vector<std::string> stringList;

    stringList.push_back( "string_one" );
    stringList.push_back( "string_two" );
    stringList.push_back( "string_three" );
    stringList.push_back( "string_four" );
    stringList.push_back( "" );
    stringList.push_back( "testing" );
    stringList.push_back( "asdfasdfasdfasdf" );

    return stringList;
}

int main( int argc, char **argv )
{
    if ( argc < 3 )
    {
        cout << "USAGE: " << argv[0] << " <dev> <baudrate>" << endl;
        exit(1);
    }

    gbxserialacfr::Serial serial( argv[1], atoi(argv[2]), gbxserialacfr::Serial::Timeout(1,0) );

    const int NUM_TIMES = 10;
    std::vector<std::string> stringList = getStringList();
    for ( int i=0; i < NUM_TIMES; i++ )
    {
        int stringI = i % (stringList.size());
        std::string sendString = stringList[stringI]+"\n";

        serial.writeString( sendString );

        std::string receiveString;
        int ret = serial.readLine( receiveString );
        if ( ret < 0 )
        {
            cout << "ERROR(serialloopbacktest.cpp): Read timed out!" << endl;
            int nBytes = serial.bytesAvailable();
            cout<<"TRACE(serialloopbacktest.cpp): bytesAvailable: " << nBytes << endl;

            if ( nBytes == 0 )
            {
                cout<<"TRACE(serialloopbacktest.cpp): Ensure that the serial port is looped-back" << endl;
                cout<<"TRACE(serialloopbacktest.cpp): (ie pins 2 & 3 connected)" << endl;                
            }
            exit(1);
        }

        if ( sendString == receiveString )
        {
            cout<<"Wrote and read: " << sendString << endl;
        }
        else
        {
            cout << "ERROR(serialloopbacktest.cpp): Strings didn't match!!" << endl;
            cout << "ERROR(serialloopbacktest.cpp): Wrote: '" << sendString <<"'"<< endl;
            cout << "ERROR(serialloopbacktest.cpp): Read:  '" << receiveString <<"'"<< endl;

            cout<<"TRACE(serialloopbacktest.cpp): test FAILED" << endl;
            exit(1);
        }
    }

    cout<<"TRACE(serialloopbacktest.cpp): test PASSED" << endl;

    return 0;
}
