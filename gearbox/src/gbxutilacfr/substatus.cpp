#include "substatus.h"
#include <iostream>

using namespace std;

namespace gbxutilacfr {

SubStatus::~SubStatus()
{
    try {
        status_.removeSubsystem( subsysName_ );
    }
    catch ( std::exception &e )
    {
        cout << "~SubStatus: exception on status_.removeSubsystem: " << e.what() << endl;
    }
    catch ( ... ) {
        cout << "~SubStatus: unknown exception on status_.removeSubsystem" << endl;
    }
}

}
