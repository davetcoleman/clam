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

#ifndef __FLEXIPORT_TYPES_H
#define __FLEXIPORT_TYPES_H

#if defined (WIN32)
	typedef unsigned char           uint8_t;
	typedef unsigned int            uint32_t;
	#if defined (_WIN64)
		typedef __int64                 ssize_t;
	#else
		typedef _W64 int                ssize_t;
	#endif
#else
	#include <stdint.h>
#endif

#endif // __FLEXIPORT_TYPES_H
