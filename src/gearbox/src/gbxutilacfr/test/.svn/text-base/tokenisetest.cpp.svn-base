#include <gbxutilacfr/tokenise.h>
#include <iostream>
#include <assert.h>
#include <cstdlib>

// Make sure assertions are on
#undef NDEBUG

using namespace std;
using namespace gbxutilacfr;

int main()
{
    {
        string str = "asdf,hjkl";
        vector<string> tokens = tokenise(str,",");
        assert( tokens[0] == "asdf" );
        assert( tokens[1] == "hjkl" );
        cout<<"TRACE(tokenisetest.cpp): str: " << str << endl;
        cout<<"TRACE(tokenisetest.cpp): tokens: " << endl;
        for ( size_t i=0; i < tokens.size(); i++) 
        {
            cout << "  " << i << ": " << tokens[i] << endl;
        }
    }

    {
        string str = ",asdf,hjkl,";
        vector<string> tokens = tokenise(str,",");
        assert( tokens[0] == "" );
        assert( tokens[1] == "asdf" );
        assert( tokens[2] == "hjkl" );
        assert( tokens[3] == "" );
        cout<<"TRACE(tokenisetest.cpp): str: " << str << endl;
        cout<<"TRACE(tokenisetest.cpp): tokens: " << endl;
        for ( size_t i=0; i < tokens.size(); i++) 
        {
            cout << "  " << i << ": " << tokens[i] << endl;
        }
    }

    {
        string str = ",asdf,,hjkl,";
        vector<string> tokens = tokenise(str,",");
        assert( tokens[0] == "" );
        assert( tokens[1] == "asdf" );
        assert( tokens[2] == "" );
        assert( tokens[3] == "hjkl" );
        assert( tokens[4] == "" );
        cout<<"TRACE(tokenisetest.cpp): str: " << str << endl;
        cout<<"TRACE(tokenisetest.cpp): tokens: " << endl;
        for ( size_t i=0; i < tokens.size(); i++) 
        {
            cout << "  " << i << ": " << tokens[i] << endl;
        }
    }

    cout<<"TRACE(tokenisetest.cpp): test PASSED" << endl;
    exit(0);
}
