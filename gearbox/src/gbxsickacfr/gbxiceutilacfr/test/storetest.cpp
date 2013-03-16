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
#include <gbxsickacfr/gbxiceutilacfr/store.h>

using namespace std;

int main(int argc, char * argv[])
{
    gbxiceutilacfr::Store<double> store;
    double data = 20.0;
    double copy = -1.0;

    cout<<"testing get() ... ";
    // call get on an empty stomach
    try
    {
        store.get( data );
        cout<<"failed. empty store, should've caught exception"<<endl;
        return EXIT_FAILURE;
    }
    catch ( const gbxutilacfr::Exception & )
    {
        ; // ok
    }
    cout<<"ok"<<endl;

    cout<<"testing peek() ... ";
    // call peek on an empty stomach
    try
    {
        store.peek( data );
        cout<<"failed. empty store, should've caught exception"<<endl;
        return EXIT_FAILURE;
    }
    catch ( const gbxutilacfr::Exception & )
    {
        ; // ok
    }
    cout<<"ok"<<endl;

    cout<<"testing getNext() ... ";
    if ( store.getNext( data, 50 )==0 ) {
        cout<<"failed. not expecting anybody setting the store"<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;
    
    cout<<"testing isEmpty() and isNewData() ... ";
    if ( !store.isEmpty() || store.isNewData() ) {
        cout<<"failed. expecting an empty non-new store."<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing set() ... ";
    for ( int i=0; i<3; ++i ) {
        store.set( data );
    }
    if ( store.isEmpty() || !store.isNewData() ) {
        cout<<"failed. expecting a non-empty new store."<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing peek() ... ";
    try
    {
        store.peek( copy );
    }
    catch ( const gbxutilacfr::Exception & )
    {
        cout<<"failed. should be a non-empty store."<<endl;
        return EXIT_FAILURE;
    }
    if ( data!=copy )
    {
        cout<<"failed. expecting an exact copy of the data."<<endl;
        cout<<"\tin="<<data<<" out="<<copy<<endl;
        return EXIT_FAILURE;
    }
    if ( store.isEmpty() || !store.isNewData() ) {
        cout<<"failed. expecting a non-empty new store."<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing get() ... ";
    try
    {
        store.get( copy );
    }
    catch ( const gbxutilacfr::Exception & )
    {
        cout<<"failed. should be a non-empty store."<<endl;
        return EXIT_FAILURE;
    }
    if ( data!=copy )
    {
        cout<<"failed. expecting an exact copy of the data."<<endl;
        cout<<"\tin="<<data<<" out="<<copy<<endl;
        return EXIT_FAILURE;
    }
    if ( store.isEmpty() || store.isNewData() ) {
        cout<<"failed. expecting a non-empty non-new store."<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    cout<<"testing getNext() ... ";
    store.set( data );
    if ( store.getNext( data, 50 )!=0 ) {
        cout<<"failed. expected to get data"<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;
    
    cout<<"testing purge()... ";
    store.purge();
    if ( !store.isEmpty() || store.isNewData() ) {
        cout<<"failed. expecting an empty non-new store."<<endl;
        return EXIT_FAILURE;
    }
    cout<<"ok"<<endl;

    
    return EXIT_SUCCESS;
}
