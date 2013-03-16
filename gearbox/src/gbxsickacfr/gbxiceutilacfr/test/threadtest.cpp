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

#include <gbxsickacfr/gbxiceutilacfr/thread.h>
#include <gbxsickacfr/gbxiceutilacfr/threadutils.h>
#include <gbxutilacfr/exceptions.h>

using namespace std;

class TestThread : public gbxiceutilacfr::Thread
{
public:    
    virtual void run()
    {
        while ( !isStopping() ) {
            IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(50));
        }
    };
};

class TestThreadWithThrow : public gbxiceutilacfr::Thread
{
public:
    TestThreadWithThrow( bool shouldIThrow )
    {
        if ( shouldIThrow ) {
            throw "throwing from constructor";
        }
    };
    
    virtual void run()
    {
        while ( !isStopping() ) {
            IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(50));
        }
    };
};

class TestThreadWithExit : public gbxiceutilacfr::Thread
{
public:  
    TestThreadWithExit() : 
        isExiting_(false) {};

    void exit()
    {
        IceUtil::Mutex::Lock lock(exitMutex_);
        isExiting_ = true;
    };

    bool isExiting()
    {
        IceUtil::Mutex::Lock lock(exitMutex_);
        return isExiting_;
    };

    virtual void run()
    {
        // this is the standard loop
        while ( !isStopping() ) {
            IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(50));
        }

        // this is a special loop for testing only
        while ( !isExiting_ ) {
            IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(50));
        }
    };

private:
    bool isExiting_;
    IceUtil::Mutex exitMutex_;
};

class TestThreadWithWait : public gbxiceutilacfr::Thread
{
public:
    virtual void run()
    {
        waitForStop();
    };
};

class TestThreadWithNap : public gbxiceutilacfr::Thread
{
public:
    virtual void run()
    {
        gbxiceutilacfr::checkedSleep( this, IceUtil::Time::seconds(5), 100 );
    };
};

int main(int argc, char * argv[])
{
    cout<<"testing start() and stop()... ";
    {
        gbxiceutilacfr::Thread* t=0;
        try
        {
            t = new TestThread;
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

    cout<<"testing start() and stop() with smart pointer ... ";
    {
        gbxiceutilacfr::ThreadPtr t;
        try
        {
            t = new TestThread;
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
    }
    cout<<"ok"<<endl;

    // alexm: is this test still needed?
    cout<<"testing Thread() with exceptions... ";
    {
        TestThreadWithThrow* t=0;
        try
        {
            t = new TestThreadWithThrow( true );
            cout<<"failed"<<endl<<"should not be able to create thread"<<endl;
            exit(EXIT_FAILURE);
        }
        catch (...)
        {
            // ok
        }
    }
    cout<<"ok"<<endl;

    cout<<"testing state machine ... ";
    {
        // works only with smart pointers because with dumb ones the threads self-destruct
        // and we can't examine their final state.
        gbxiceutilacfr::ThreadPtr t = new TestThreadWithExit;

        if ( t->isStopping()!=false || t->isAlive()!=false || t->isStarted()!=false ) {
            cout<<"failed"<<endl
                <<"should be in Starting state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }

        t->start();
        if ( t->isStopping()!=false || t->isAlive()!=true || t->isStarted()!=true ) {
            cout<<"failed"<<endl
                <<"should be in Started state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }

        t->stop();
        if ( t->isStopping()!=true || t->isAlive()!=true || t->isStarted()!=true ) {
            cout<<"failed"<<endl
                <<"should be in Stopping state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }

        IceUtil::ThreadControl tc = t->getThreadControl();
        // this ugly shit is to call one special function
        TestThreadWithExit* dumb = (TestThreadWithExit*)&(*t);
        dumb->exit();
        IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(150));
        // alexm: I think it self-destructs here. what can we tell about it?
        if ( t->isStopping()!=true || t->isAlive()!=false || t->isStarted()!=true ) {
            cout<<"failed"<<endl
                <<"should be in Stopped state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }
        tc.join();
    }
    cout<<"ok"<<endl;
    
    cout<<"testing waitForStop() ... ";
    {
        gbxiceutilacfr::ThreadPtr t = new TestThreadWithWait;
        t->start();
        if ( t->isAlive()!=true ) {
            cout<<"failed"<<endl
                <<"should be in Started state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }

        t->stop();
        IceUtil::ThreadControl tc = t->getThreadControl();
        IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(150));
        if ( t->isAlive()!=false ) {
            cout<<"failed"<<endl
                <<"should be in Stopped state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }
        tc.join();
    }
    cout<<"ok"<<endl;

    // alexm: don't know how to test this, because of self-destruction
//     cout<<"testing stopAndJoin() ... ";
//     {
//     }
//     cout<<"ok"<<endl;

    cout<<"testing stopAndJoin() with smart pointer ... ";
    {
        gbxiceutilacfr::ThreadPtr t = new TestThread;
        t->start();
        if ( t->isAlive()!=true ) {
            cout<<"failed"<<endl
                <<"should be in Started state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }

        gbxiceutilacfr::stopAndJoin( t );
        IceUtil::ThreadControl::sleep(IceUtil::Time::milliSeconds(150));
        if ( t->isAlive()!=false ) {
            cout<<"failed"<<endl
                <<"should be in Stopped state but internal states do not match:"<<endl
                <<"isStopping="<<(int)t->isStopping()<<" isAlive="<<(int)t->isAlive()<<" isStarted()="<<(int)t->isStarted()<<endl;
            exit(EXIT_FAILURE);
        }
    }
    cout<<"ok"<<endl;

    cout<<"testing checkedSleep() ... ";
    {
        gbxiceutilacfr::ThreadPtr t = new TestThreadWithNap;
        t->start();

        IceUtil::Time stopTime = IceUtil::Time::now();
        gbxiceutilacfr::stopAndJoin( t );
        IceUtil::Time joinTime = IceUtil::Time::now();
        cout<<"time to stop = "<<(joinTime-stopTime).toDuration()<<endl;

        if ( joinTime-stopTime > IceUtil::Time::seconds(1) ) {
            cout<<"failed"<<endl
                <<"should stop faster than in 1 second, time to stop ="<<(joinTime-stopTime).toDuration()<<endl;
            exit(EXIT_FAILURE);
        }
    }
    cout<<"ok"<<endl;

    return EXIT_SUCCESS;
}
