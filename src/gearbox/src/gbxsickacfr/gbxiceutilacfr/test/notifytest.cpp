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
#include <gbxsickacfr/gbxiceutilacfr/notify.h>

using namespace std;

class TestNotifyHandler : public gbxiceutilacfr::NotifyHandler<double>
{
public:
    virtual void handleData( const double& obj )
    {
        copy_=obj;
    };

    double copy_;
};

int main(int argc, char * argv[])
{
    gbxiceutilacfr::Notify<double> notify;
    double data = 2.0;
    
    gbxiceutilacfr::NotifyHandler<double>* emptyHandler = 0;
    TestNotifyHandler testHandler;

    cout<<"testing set() ... ";
    // call set on an empty stomach
    try
    {
        notify.set( data );
        cout<<"failed. empty notify handler, should've caught exception"<<endl;
        return EXIT_FAILURE;
    }
    catch ( const gbxutilacfr::Exception & )
    {
        ; // ok
    }
    cout<<"ok"<<endl;

    cout<<"testing hasNotifyHandler() ... ";
    if ( notify.hasNotifyHandler()!=0 ) {
        cout<<"failed. not expecting to have a handler"<<endl;
        return EXIT_FAILURE;
    }
    notify.setNotifyHandler( emptyHandler );
    if ( notify.hasNotifyHandler()!=0 ) {
        cout<<"failed. still not expecting to have a handler"<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing setNotifyHandler() ... ";
    notify.setNotifyHandler( &testHandler );
    if ( notify.hasNotifyHandler()==0 ) {
        cout<<"failed. expecting to have a handler"<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing set() ... ";
    try
    {
        notify.set( data );
    }
    catch ( const gbxutilacfr::Exception & )
    {
        cout<<"failed. shouldn't have caught exception"<<endl;
        return EXIT_FAILURE;
    }
    if ( data != testHandler.copy_ ) {
        cout<<"failed. expecting an exact copy of the data."<<endl;
        cout<<"\tin="<<data<<" out="<<testHandler.copy_<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;
    
    return EXIT_SUCCESS;
}
