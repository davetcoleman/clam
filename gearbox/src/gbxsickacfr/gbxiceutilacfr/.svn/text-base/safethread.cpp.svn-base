/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <sstream>

#include "safethread.h"

using namespace gbxiceutilacfr;
using namespace std;


SafeThread::SafeThread( gbxutilacfr::Tracer& tracer ) :
    tracer_(tracer)
{
}

void 
SafeThread::run()
{
    stringstream ss;
    try
    {
        walk();
    }
    catch ( const IceUtil::Exception &e ) {
        ss << "SafeThread::run(): Caught unexpected exception: " << e;
    }
    catch ( const std::exception &e ) {
        ss << "SafeThread::run(): Caught unexpected exception: " << e.what();
    }
    catch ( const std::string &e ) {
        ss << "SafeThread::run(): Caught unexpected string: " << e;
    }
    catch ( const char *e ) {
        ss << "SafeThread::run(): Caught unexpected char *: " << e;
    }
    catch ( ... ) {
        ss << "SafeThread::run(): Caught unexpected unknown exception.";
    }

    // report error if there was an exception and we are not stopping
    if ( !ss.str().empty() ) {
        if ( !isStopping() ) 
            tracer_.error( ss.str() );
        else
            tracer_.debug( ss.str(), 4 );
    }
    else {
        tracer_.debug( "SafeThread: dropping out from run()", 4 );
    }

    // wait for the component to realize that we are quitting and tell us to stop.
    waitForStop();
}
