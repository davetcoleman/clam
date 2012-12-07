#include <iostream>
#include <sstream>

using namespace std;

template < typename T >
inline T highbit(T& t)
{
    return t = (((T)(-1)) >> 1) + 1;
}

template < typename T >
std::ostream& bin(T& value, std::ostream &o)
{
    for ( T bit = highbit(bit); bit; bit >>= 1 )
    {
        o << ( ( value & bit ) ? '1' : '0' );
    }
    return o;
}

// Simple checksum test using XOR
int main( int argc, char **argv )
{
    int opt;
    std::string input = "B18,0A,0000";
    std::string resultHex = "3A";
    
    // Get some options from the command line
    while ((opt = getopt(argc, argv, "i:r:")) != -1)
    {
        switch ( opt )
        {
            case 'i':
                input = optarg;
                break;
            case 'r':
                resultHex = optarg;
                break;
            default:
                cout << "Usage: " << argv[0] << " [-i input] [-r resultInHex]" << endl
                     << "-i input\tInput string. E.g. B18,0A,0000" << endl
                     << "-r resultInHex\tResult in hex. E.g. 3A "<< endl;
                return 1;
        }
    }

    cout << endl << "Checksum is computed using XOR" << endl;
    cout << endl << "Input string is:\t\t " << input << endl;
    cout << "Expected checksum result is:\t " << resultHex << endl << endl;

    unsigned int checksum = 0;

    cout << "Table: " << endl
         << "char\tdec\thex\tbin" << endl
         << "----------------------------------" << endl;

    for (unsigned int i=0; i<input.size(); i++)
    {
        unsigned int charValue = (unsigned int)input[i];
        cout << input[i] << "\t" 
             << std::dec << charValue << "\t" 
             << std::hex << charValue << "\t";
        bin(charValue, cout);
        cout << endl;
        //checksum computation
        checksum ^= charValue;
    }

    cout << endl;
    cout << "Computed checksum (dec,hex,bin): " << std::dec << checksum << ", " << std::hex << checksum << ", ";
    bin(checksum, cout); cout << endl << endl;

    stringstream ss;
    ss << std::hex << checksum;

    string checksumStr = ss.str();
    for (unsigned int i=0; i<checksumStr.size(); i++)
        checksumStr[i] = (char)(toupper( checksumStr[i] ));

    cout << "Expected checksum result is:\t" << resultHex << endl;
    cout << "Computed checksum result is:\t" << checksumStr << endl << endl;

    int passTest = checksumStr.compare( resultHex );
    if (passTest!=0) {
        cout << "Test not passed" << endl;
        return 1;
    }
    cout << "Test passed" << endl;
    
    return 0;
}


