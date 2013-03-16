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

#include <IceUtil/Time.h>

#include <gbxsickacfr/gbxiceutilacfr/safethread.h>
#include <gbxutilacfr/trivialtracer.h>

using namespace std;

class TestThread : public gbxiceutilacfr::SafeThread
{
public:    
    // it's safe to pass zero pointers
    TestThread( gbxutilacfr::Tracer& tracer ) : 
        SafeThread( tracer ) {};
    virtual void walk()
    {
        while ( !isStopping() ) {
            IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(50));
        }
    };
};

class TestThreadWithThrow : public gbxiceutilacfr::SafeThread
{
public:
    // it's safe to pass zero pointers
    TestThreadWithThrow( gbxutilacfr::Tracer& tracer ) : 
        SafeThread( tracer ) {};
    virtual void walk()
    {
        throw "throwing from walk";
    };
};

int main(int argc, char * argv[])
{
    gbxutilacfr::TrivialTracer tracer;

    cout<<"testing start() and stop()... ";
    {
        gbxiceutilacfr::Thread* t=0;
        try
        {
            t = new TestThread( tracer );
            t->start();
        }
        catch (...)
        {
            cout<<"failed"<<endl<<"should be able to create thread"<<endl;
            exit(EXIT_FAILURE);
        }
        IceUtil::ThreadControl tc = t->getThreadControl();
        t->stop();
        tc.join();
        // do not delete t! it's already self-destructed.
    }
    cout<<"ok"<<endl;

    cout<<"testing SafeThread() with exceptions... ";
    {
        gbxiceutilacfr::Thread* t=0;
        try
        {
            t = new TestThreadWithThrow( tracer );
            t->start();
        }
        catch (...)
        {
            cout<<"failed"<<endl<<"all exception should've been caught."<<endl;
            // ok
        }
        IceUtil::ThreadControl tc = t->getThreadControl();
        t->stop();
        tc.join();
        // do not delete t! it's already self-destructed.
    }
    cout<<"ok"<<endl;

    return EXIT_SUCCESS;
}
