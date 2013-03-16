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

#ifndef __FLEXIPORT_H
#define __FLEXIPORT_H

#include <map>
#include <string>

#if defined (WIN32)
	#if defined (FLEXIPORT_STATIC)
		#define FLEXIPORT_EXPORT
	#elif defined (FLEXIPORT_EXPORTS)
		#define FLEXIPORT_EXPORT    __declspec (dllexport)
	#else
		#define FLEXIPORT_EXPORT    __declspec (dllimport)
	#endif
#else
	#define FLEXIPORT_EXPORT
#endif

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/// @brief Exception thrown by the Port class and its derivatives.
class FLEXIPORT_EXPORT PortException : public std::exception
{
	public:
		PortException (const char *errorStr)
			: _errorStr (errorStr)	{};
		PortException (const std::string &errorStr)
			: _errorStr (errorStr)	{};
		~PortException () throw () {};

		virtual const char* what () const throw ()
		{
			return _errorStr.c_str ();
		}

	private:
		std::string _errorStr;
};

// Forward declaration of the Port class
class Port;

/** @brief Factory function to create a @ref Port object of the necessary port type.

This factory function is the only way to create a @ref Port object matching the type of port you
need. Options are passed in as key/value pairs in the @ref options argument. A minimum of one option
is necessary, the "type" option. This specifies the type of port to be created. An object of the
correct class (@ref SerialPort, @ref TCPPort or @ref LogReaderPort, for example) will be created
based on the value of this option. All other options, including both universal options and
type-specific options, will be passed on to the created object.

See @ref Options for a list of options accepted by all port types. See each port type's description
for a list of options accepted by that port type.

@return A pointer to a @ref Port object.*/
FLEXIPORT_EXPORT Port* CreatePort (std::map<std::string, std::string> options);

/** @brief Overloaded factory function. Accepts options as a string.

This overload accepts the options as a string. For example:
"type=serial device=/dev/ttyS0 baud=4800 readwrite"
Options may be separated by any of: spaces, tabs, new lines or commas. Do not put whitespace around
an equals sign.

@return A pointer to a @ref Port object.*/
FLEXIPORT_EXPORT Port* CreatePort (std::string options);

} // namespace flexiport

/** @} */

#endif // __FLEXIPORT_H
