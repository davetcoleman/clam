/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <gbxsickacfr/gbxiceutilacfr/timer.h>

namespace gbxiceutilacfr {

Timer::Timer() :
    startTime_( IceUtil::Time::now() )
{
}

Timer::Timer( const IceUtil::Time& elapsedTime ) :
    startTime_( IceUtil::Time::now()-elapsedTime )
{
}

void Timer::restart()
{
    startTime_ = IceUtil::Time::now();
}

IceUtil::Time Timer::elapsed() const
{
    return IceUtil::Time::now() - startTime_;
}

double Timer::elapsedMs() const
{
    return ( (IceUtil::Time::now() - startTime_).toMilliSecondsDouble() );
}

double Timer::elapsedSec() const
{
    return ( (IceUtil::Time::now() - startTime_).toSecondsDouble() );
}

} // namespace
