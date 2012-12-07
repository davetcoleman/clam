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

#ifndef __LOGREADERPORT_H
#define __LOGREADERPORT_H

#include "port.h"

#include <map>
#include <string>

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

class LogFile;

/** @brief Simulated port using a log file.

Uses a log file created by the @ref LogWriterPort port type to simulate the data transfer over a
@ref Port object.

@note Log files greater than 2GB in size are not supported.

@note The timer resolution under Windows is milliseconds, not microseconds. This may result in
inaccurate replay when using a log file created on a POSIX-compatible operating system.

See the @ref Port class documentation for how to use the common API.

@par Options
 - file <string>
   - File name to read the log from. Two files will be created using this name and a suffix.
   - Default: port.log
 - ignoretimes
   - Ignore time stamps in the log files. This means that all readable data is available instantly.
     It also overrides strictness level 2, if set, essentially turning it into strictness level 1.
   - Default: false
 - strictness <integer>
   - Level of strictness to require:
     - 0: Writes are not checked against the log file.
     - 1: Writes are checked against the log file, but timing is not checked.
     - 2: Writes are checked against the log file. Timing is checked; if a write occurs earlier than
       did in the log file, it will cause an error. Jitter can be set to allow a margin of error.
   - Default: 0
 - jitter <integer>
   - Margin of error (in milliseconds) to allow for write checking with strictness = 2.
   - Default: 100 */
class FLEXIPORT_EXPORT LogReaderPort : public Port
{
	public:
		LogReaderPort (std::map<std::string, std::string> options);
		~LogReaderPort ();

		/// @brief Open the port.
		void Open ();
		/// @brief Close the port.
		void Close ();
		/// @brief Read from the port.
		ssize_t Read (void * const buffer, size_t count);
		/// @brief Read the requested quantity of data from the port.
		ssize_t ReadFull (void * const buffer, size_t count);
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
		LogFile *_logFile;
		std::string _logFileName;
		unsigned int _strictness;
		int _jitter;
		bool _ignoreTimes;
		bool _open;

		bool ProcessOption (const std::string &option, const std::string &value);
		void CheckPort (bool read);
};

} // namespace flexiport

/** @} */

#endif // __LOGREADERPORT_H
