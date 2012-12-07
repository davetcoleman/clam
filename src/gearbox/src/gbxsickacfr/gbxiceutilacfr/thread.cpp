/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <IceUtil/Time.h>
#include <gbxutilacfr/exceptions.h>

#include "thread.h"

namespace gbxiceutilacfr {

Thread::Thread() : 
    isStopping_(false) 
{
}

bool 
Thread::isStarted()
{
    // must use this mutex from IceUtil::Thread
    IceUtil::Mutex::Lock lock(_stateMutex);
    return _started;
}

void 
Thread::stop()
{
    // using the mutex from IceUtil::Thread for convenience
    IceUtil::Mutex::Lock lock(_stateMutex);
    isStopping_ = true;
}

bool 
Thread::isStopping()
{
    // using the mutex from IceUtil::Thread for convenience
    IceUtil::Mutex::Lock lock(_stateMutex);
    return isStopping_;
}

void 
Thread::waitForStop()
{
    while ( !isStopping() ) {
        IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(10));
    }
}

void stop( gbxiceutilacfr::Thread* thread )
{
    try {
        if ( thread ) {
            // get the control object first
            IceUtil::ThreadControl tc = thread->getThreadControl();
        
            // Tell the thread to stop
            thread->stop();
        }
    }
    catch ( IceUtil::ThreadNotStartedException )
    {
        // Catch this and quietly ignore it -- threads are often
        // stopped in destructors, we don't wanna be throwing
        // exceptions.
    }
}

void stopAndJoin( gbxiceutilacfr::Thread* thread )
{
    try {
        if ( thread ) {
            // get the control object first
            IceUtil::ThreadControl tc = thread->getThreadControl();
        
            // Tell the thread to stop
            thread->stop();
    
            // Then wait for it
            tc.join();
        }
    }
    catch ( IceUtil::ThreadNotStartedException )
    {
        // Catch this and quietly ignore it -- threads are often
        // stopped in destructors, we don't wanna be throwing
        // exceptions.
    }
}

} // namespace
