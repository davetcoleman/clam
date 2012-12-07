/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXICEUTILACFR_SAFE_THREAD_H
#define GBXICEUTILACFR_SAFE_THREAD_H

#include <gbxsickacfr/gbxiceutilacfr/thread.h>
#include <gbxutilacfr/tracer.h>

namespace gbxiceutilacfr {

/*!
@brief A version of the Thread class which catches all possible exceptions.

If an exception is caught when the thread is not stopping, an error trace message 
will be printed. Then the thread wil wait for someone to call stop().

To use this class, simply implement the pure virtual walk() function.
@verbatim
void MyThread::walk()
{
    // initialize

    // main loop
    while ( !isStopping() )
    {
        // do something
    }

    // clean up
}
@endverbatim

@see Thread, SubsystemThread.
 */
class SafeThread : public Thread
{
public:
    //! Needs an implementation of Tracer to report possible exceptions.
    SafeThread( gbxutilacfr::Tracer& tracer );

    // from IceUtil::Thread (from which HydroUtil::Thread is derived)
    //! This implementation calls walk(), catches all possible exceptions, prints out 
    //! errors and waits for someone to call stop().
    virtual void run();

private:
    //! Implement this function in the derived class and put here all the stuff which your
    //! thread needs to do.
    virtual void walk()=0;

    gbxutilacfr::Tracer& tracer_;
};
//! A smart pointer to the SafeThread class.
typedef IceUtil::Handle<SafeThread> SafeThreadPtr;

} // end namespace

#endif
