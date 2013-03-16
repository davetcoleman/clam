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

#ifndef __LOGWRITERPORT_H
#define __LOGWRITERPORT_H

#include "port.h"

#include <map>
#include <string>

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

class LogFile;

/** @brief Logging implementation of the @ref Port class. An underlying Port object is used to
perform the actual communications. All actions taken by that port are logged, including data
transferred. The log file can be used with a LogReaderPort to simulate a Port object and perform
testing.

To create a LogWriterPort, append "log" to the port type in the port options passed to
@ref CreatePort. For example, a log writer that uses a serial port would be specified as the type
"seriallog". Similarly, for a TCP port, use "tcplog". 

@note Log files greater than 2GB in size are not supported.

@note The timer resolution under Windows is milliseconds, not microseconds. This may result in
inaccurate replay when using a log file created on a POSIX-compatible operating system.

See the @ref Port class documentation for how to use the common API.

@par Options
 - file <string>
   - File name to save the log to.
   - Default: port.log

All unused options will be passed on to the underlying port used.
*/
class FLEXIPORT_EXPORT LogWriterPort : public Port
{
	public:
		LogWriterPort (std::map<std::string, std::string> options);
		~LogWriterPort ();

		/// @brief Open the port.
		void Open ();
		/// @brief Close the port.
		void Close ();
		/// @brief Read from the port.
		ssize_t Read (void * const buffer, size_t count);
		/// @brief Read the requested quantity of data from the port.
		ssize_t ReadFull (void * const buffer, size_t count);
		/// @brief Dump data until the specified number of bytes have been read.
		ssize_t Skip (size_t count);
		/** @brief Read and dump data until the specified termination character has been seen @ref
		count times. */
		ssize_t SkipUntil (uint8_t terminator, unsigned int count);
		/// @brief Get the number of bytes waiting to be read at the port. Returns immediatly.
		ssize_t BytesAvailable ();
		/// @brief Get the number of bytes waiting after blocking for the timeout.
		ssize_t BytesAvailableWait ();
		/// @brief Write data to the port.
		ssize_t Write (const void * const buffer, size_t count);
		/// @brief Flush the port's input and output buffers, discarding all data.
		void Flush ();
		/// @brief Drain the port's input and output buffers.
		void Drain ();
		/// @brief Get the status of the port (type, device, etc).
		std::string GetStatus () const;
		/// @brief Set the timeout value in milliseconds.
		void SetTimeout (Timeout timeout);
		/// @brief Set the read permissions of the port.
		void SetCanRead (bool canRead);
		/// @brief Set the write permissions of the port.
		void SetCanWrite (bool canWrite);
		/// @brief Check if the port is open
		bool IsOpen () const;

	private:
		Port *_port;
		LogFile *_logFile;
		std::string _logFileName;

		void CheckPort (bool read);
};

} // namespace flexiport

/** @} */

#endif // __LOGWRITERPORT_H
