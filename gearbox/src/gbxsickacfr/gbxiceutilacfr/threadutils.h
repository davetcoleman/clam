/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXICEUTILACFR_THREAD_UTILS_H
#define GBXICEUTILACFR_THREAD_UTILS_H

#include <IceUtil/Time.h>
#include <gbxutilacfr/stoppable.h>

namespace gbxiceutilacfr {

//! Sleeps for @ref duration waking up every @ref checkIntervalMs [ms] to check if the @c activity 
//! was told to stop. This implementation is very simple so the error in total sleep duration 
//! can be as large as checkIntervalMs. In particular, if @ref duration is shorter than @ref checkIntervalMs,
//! this function will sleep for @ref checkIntervalMs.
void checkedSleep( gbxutilacfr::Stoppable* activity, const IceUtil::Time& duration, int checkIntervalMs=250 );

//! Same as above, but sleep duration is specified in milliseconds.
void checkedSleep( gbxutilacfr::Stoppable* activity, int durationMs, int checkIntervalMs=250 );

} // end namespace

#endif
