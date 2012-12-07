/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <assert.h>
#include <IceUtil/Thread.h>
#include "threadutils.h"

namespace gbxiceutilacfr {

void checkedSleep( gbxutilacfr::Stoppable* activity, const IceUtil::Time& duration, int checkIntervalMs  )
{
    assert( activity && "Null activity pointer" );

    // Handle durations less than the check interval
    if ( duration.toMilliSeconds() < checkIntervalMs )
    {
        IceUtil::ThreadControl::sleep( duration );
        return;
    }

    IceUtil::Time wakeupTime = IceUtil::Time::now() + duration;
    IceUtil::Time checkInterval = IceUtil::Time::milliSeconds( checkIntervalMs );

    while ( !activity->isStopping() && IceUtil::Time::now() < wakeupTime )
    {
        IceUtil::ThreadControl::sleep( checkInterval );
    }
}

void checkedSleep( gbxutilacfr::Stoppable* activity, int durationMs, int checkIntervalMs  )
{
    checkedSleep( activity, IceUtil::Time::milliSeconds( durationMs ), checkIntervalMs );
}

} // namespace
