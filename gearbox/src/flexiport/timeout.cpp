/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2008 Geoffrey Biggs
 *
 * flexiport flexible hardware data communications library.
 * 
 * This distribution is licensed to you under the terms described in the LICENSE file included in 
 * this distribution.
 *
 * This work is a product of the National Institute of Advanced Industrial Science and Technology,
 * Japan. Registration number: H20PRO-881
 * 
 * This file is part of flexiport.
 *
 * flexiport is free software: you can redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software Foundation, either version 3 of 
 * the License, or (at your option) any later version.
 *
 * flexiport is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with flexiport.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#if defined (WIN32)
	#include <time.h>
	// timeval is only in WinSock (stupidity)
	#include <winsock2.h>
#else
	#include <sys/time.h>
#endif

#include "timeout.h"

namespace flexiport
{

void Timeout::AsTimeval (struct timeval &dest) const
{
	dest.tv_sec = _sec;
	dest.tv_usec = _usec;
}

void Timeout::FromTimeval (const struct timeval &src)
{
	_sec = src.tv_sec;
	_usec = src.tv_usec;
}

void Timeout::AsTimespec (struct timespec &dest) const
{
	dest.tv_sec = _sec;
	dest.tv_nsec = _usec * 1000;
}

void Timeout::FromTimespec (const struct timespec &src)
{
	_sec = src.tv_sec;
	_usec = src.tv_nsec / 1000;
}

Timeout& Timeout::operator= (const Timeout &rhs)
{
	if (this == &rhs)
		return *this;

	_sec = rhs._sec;
	_usec = rhs._usec;
	return *this;
}

Timeout& Timeout::operator= (const struct timeval &rhs)
{
	_sec = rhs.tv_sec;
	_usec = rhs.tv_usec;
	return *this;
}

Timeout& Timeout::operator= (const struct timespec &rhs)
{
	_sec = rhs.tv_sec;
	_usec = rhs.tv_nsec / 1000;
	return *this;
}

} // namespace flexiport
