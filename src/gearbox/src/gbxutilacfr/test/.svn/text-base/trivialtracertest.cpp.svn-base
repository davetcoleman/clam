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
#include <cstdlib>
#include <gbxutilacfr/trivialtracer.h>

using namespace std;

int main(int argc, char * argv[])
{    
    cout<<"testing TrivialTracer() without props ..."<<endl;
    gbxutilacfr::TrivialTracer tracer;
    cout<<"ok"<<endl;

    string message = "when you don't have anything nice to say...";

    tracer.print( message );
    tracer.info( message );
    tracer.warning( message );
    tracer.error( message );
    tracer.debug( message );

    return EXIT_SUCCESS;
}
