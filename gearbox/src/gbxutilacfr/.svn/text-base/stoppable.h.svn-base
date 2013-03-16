/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_STOPPABLE_H
#define GBXUTILACFR_STOPPABLE_H

#if defined (WIN32)
    #if defined (GBXUTILACFR_STATIC)
        #define GBXUTILACFR_EXPORT
    #elif defined (GBXUTILACFR_EXPORTS)
        #define GBXUTILACFR_EXPORT       __declspec (dllexport)
    #else
        #define GBXUTILACFR_EXPORT       __declspec (dllimport)
    #endif
#else
    #define GBXUTILACFR_EXPORT
#endif

namespace gbxutilacfr {

/*!
@brief An abstract interface class representing an stoppable activity.  

Inherit from this and other classes will know how to ask if your class was stopped (interrupted).

@verbatim
function doBigJob( gbxutilacfr::Stoppable* parent )
{
    // check periodically that the parent activity is not stopping
    while( !parent->isStopping() )
    {
        // perform many steps
    }
}

class MyActivity : public gbxutilacfr::Stoppable
{
public:
    virtual bool isStopping()
    {
        // return TRUE if we are interrupted
    };

    void run()
    {
        doBigJob( this );
    };
};
@endverbatim 

@author Alex Makarenko
*/
class GBXUTILACFR_EXPORT Stoppable
{
public:
    virtual ~Stoppable() {};

    virtual bool isStopping()=0;
};

} // namespace

#endif
