#include <iostream>
#include <gbxserialacfr/serial.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip> 
#include <assert.h>

using namespace std;

//
// A simple utility program for testing serial ports.
//   Listens for incoming data, echoes it to the terminal.
//

std::string
toHexString( const std::vector<unsigned char> &data )
{
    stringstream ss;
    ss << "[ ";
    for ( size_t i=0; i < data.size(); i++ )
    {
        ss <<hex<<std::setfill('0')<<std::setw(2)<<(int)(data[i])<<" ";
    }
    ss << "]";
    return ss.str();    
}

int main( int argc, char **argv )
{
    if ( argc < 3 )
    {
        cout << "USAGE: " << argv[0] << " <dev> <baudrate>" << endl;
        exit(1);
    }

    gbxserialacfr::Serial serial( argv[1], atoi(argv[2]), gbxserialacfr::Serial::Timeout(1,0) );

    std::vector<unsigned char> data;
    while ( true )
    {
        cout<<"TRACE(serialechotest.cpp): Waiting for data..." << endl;
        size_t nBytes = serial.bytesAvailableWait();

        if ( nBytes > 0 )
        {
            cout<<"TRACE(serialechotest.cpp): nBytes: " << nBytes << endl;
            if ( nBytes > data.size() )
            {
                cout<<"TRACE(serialechotest.cpp): resizing to " << nBytes << endl;
                data.resize( nBytes );
            }

//             cout<<"TRACE(serialechotest.cpp): data.size(): " << data.size() << endl;
//             cout<<"TRACE(serialechotest.cpp): &(data[0]): " << (int)(&(data[0])) << endl;
//             cout<<"TRACE(serialechotest.cpp): data[0]: " << (int)(data[0]) << endl;

            int numRead = serial.read( &(data[0]), nBytes );
            // There were nBytes available, we should be able to read them all
            assert( numRead == (int)nBytes );

            cout << "got data: " << toHexString( data ) << endl;
        }
    }

    return 0;
}
