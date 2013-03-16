/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <cstdlib>
#include <gbxserialacfr/lockfile/lockfile.h>

using namespace std;

int main( int argc, char **argv )
{
    const char *dev = "/dev/ttyS0";

    gbxserialacfr::lockfile::LockFile *lock;

    try {
        lock = new gbxserialacfr::lockfile::LockFile( dev );
    }
    catch ( const std::exception &e )
    {
        cout<<"TRACE(locktest.cpp): Failed to lock: " << e.what() << endl;
        exit(1);
    }

    try {
        gbxserialacfr::lockfile::LockFile secondLock( dev );
        
        cerr << "Error: double-lock succeeded.";
        exit(1);
    }
    catch ( const std::exception &e )
    {
        cout<<"TRACE(locktest.cpp): Correctly caught exception: " << e.what() << endl;
    }

    delete lock;

    cout<<"TRACE(locktest.cpp): test PASSED!" << endl;
    return 0;
}
