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

#ifndef __TIMEOUT_H
#define __TIMEOUT_H

#if defined (WIN32)
	#if defined (FLEXIPORT_STATIC)
		#define FLEXIPORT_EXPORT
	#elif defined (FLEXIPORT_EXPORTS)
		#define FLEXIPORT_EXPORT    __declspec (dllexport)
	#else
		#define FLEXIPORT_EXPORT    __declspec (dllimport)
	#endif
	#if !defined (timespec)
		// No timespec on Windows
		typedef struct timespec
		{
			int tv_sec;
			int tv_nsec;
		} timespec;
	#endif
#else
	#define FLEXIPORT_EXPORT
#endif

struct timeval;
struct timespec;

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/** @brief An object used to represent timeouts. */
class FLEXIPORT_EXPORT Timeout
{
	public:
		Timeout (int sec, int usec) : _sec (sec), _usec (usec) {}
		Timeout (const Timeout &rhs) : _sec (rhs._sec), _usec (rhs._usec) {}

		void AsTimeval (struct timeval &dest) const;
		void FromTimeval (const struct timeval &src);
		void AsTimespec (struct timespec &dest) const;
		void FromTimespec (const struct timespec &src);

		Timeout& operator= (const Timeout &rhs);
		Timeout& operator= (const struct timeval &rhs);
		Timeout& operator= (const struct timespec &rhs);

		int _sec;
		int _usec;
};

} // namespace flexiport

/** @} */

#endif // __TIMEOUT_H
