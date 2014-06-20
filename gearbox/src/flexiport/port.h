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

#ifndef __PORT_H
#define __PORT_H

#include <string>
#include <map>

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

#include "flexiport_types.h"
#include "timeout.h"
#include <sys/types.h>

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/**
@brief Base Port class.

This provides the base object from which specific port type implementations inherit.

All functions may throw exceptions of type @ref PortException.

@par Options
 - debug <integer>
   - Debug level. Higher numbers give more information.
   - Default: 0 (no debug information)
 - timeout <float>
   - Time out on read/write, in seconds.microseconds.
   - A timeout of -1 disables timeouts, creating a port that will block forever.
   - Default: -1
 - readonly, writeonly, readwrite
   - Specify one type of permissions.
   - Default: readwrite
 - alwaysopen
   - The port should be open for as long as the object exists. It will be opened when constructed
     and if it closes unexpectedly, an attempt will be made to reopen it.
   - Default: off */
class FLEXIPORT_EXPORT Port
{
	public:
		virtual ~Port ();

		// API common to all ports
		/// @brief Open the port.
		virtual void Open () = 0;

		/// @brief Close the port.
		virtual void Close () = 0;

		/** @brief Read from the port.

		Reads up to @ref count bytes from the port into @ref buffer.
		The exact behaviour of this function depends on the value of @ref _timeout.
		- When the timeout is non-zero, it will block until data is received or the timeout occurs.
		- When the timeout is zero, it will timeout immediatly if no data is available.
		- When the timeout is -1, it will block forever.

		@return The number of bytes actually read, or -1 if a timeout occured. If zero is returned,
		this indicates that the port closed (and possibly reopened if set to do so).*/
		virtual ssize_t Read (void * const buffer, size_t count) = 0;

		/** @brief Read the requested quantity of data from the port.

		Reads @ref count bytes from the port into @ref buffer.
		Similar to @ref Read, but with the important difference that, rather than returning as soon
		as any data is received, it will continue trying to receive data until @ref buffer is full
		or an error occurs.
		This function ignores the timeout setting and blocks anyway.

		@return The number of bytes actually read. Timeouts will cause an exception (because they
		shouldn't happen). */
		virtual ssize_t ReadFull (void * const buffer, size_t count) = 0;

		/** @brief Read a string.

		A convenience function that reads data from the port and returns it in a string. Behaves
		the same as @ref Read.

		@return The length of the string, or -1 if a timeout occured. */
		virtual ssize_t ReadString (std::string &buffer);

		/** @brief Read data until a specified termination byte is received.

		Reads up to @ref count bytes from the port into @ref buffer, stopping when a byte matching
		@ref terminator is received (included in the returned data). Otherwise behaves the same as
		@ref Read.

		@note This function makes many calls to Read, each of which has an individual timeout. The
		maximum length of time this function make take may therefore be longer than one timeout.

		@note If the port is set to non-blocking mode (by setting the timeout to zero), this will
		effectively timeout immediatly when there is no data available, returning -1 irrespective
		of the quantity of data actually received before that point.

		@return The number of bytes actually read (including the terminator), or -1 if a timeout
		occured. */
		virtual ssize_t ReadUntil (void * const buffer, size_t count, uint8_t terminator);

		/** @brief Read a string until the specified termination character is received.

		A convenience function that is similar to @ref ReadUntil. The result is stored in a string.
		The terminator character is included in the returned string. Good for text-based protocols
		with a known message termination character.

		@note This function makes many calls to Read, each of which has an individual timeout. The
		maximum length of time this function make take may therefore be longer than one timeout.

		@note If the port is set to non-blocking mode (by setting the timeout to zero), this will
		effectively timeout immediatly when there is no data available, returning -1 irrespective
		of the quantity of data actually received before that point.

		@return The number of bytes actually read (including the terminator), or -1 if a timeout
		occured. */
		virtual ssize_t ReadStringUntil (std::string &buffer, char terminator);

		/** @brief Read a new-line terminated string of data.

		A convenience function that reads until a newline character (\\n, 0x0A) is received and
		stores the received data in a caller-provided buffer, @ref buffer. Good for text-based
		protocols that use newlines as message terminators. Will not read more than @ref count
		bytes.

		@ref buffer should include space for a NULL byte. @ref count should reflect this. e.g. if
		you are expecting to receive a string "abcd\n", you should send a buffer that is 6 bytes
		long and make count = 6. This function will take into account the need for a NULL terminator
		when it receives data, receiving at most one less than count bytes. This NULL byte will not
		be included in the length of the received string returned from this function (just like
		strlen ()).

		@note This function makes many calls to Read, each of which has an individual timeout. The
		maximum length of time this function may take may therefore be longer than one timeout.

		@note If the port is set to non-blocking mode (by setting the timeout to zero), this will
		effectively timeout immediately when there is no data available, returning -1 irrespective of
		the quantity of data actually received before that point.

		@return The length of the string (including the new line), or -1 if a timeout occured. */
		virtual ssize_t ReadLine (char * const buffer, size_t count);

		/** @brief Read a new-line terminated string of data.

		A convenience function that reads until a newline character (\\n, 0x0A) is received and
		stores the received data in a string, @buffer. Good for text-based protocols that use
		newlines as message terminators.

		@note This function makes many calls to Read, each of which has an individual timeout. The
		maximum length of time this function make take may therefore be longer than one timeout.

		@note If the port is set to non-blocking mode (by setting the timeout to zero), this will
		effectively timeout immediatly when there is no data available, returning -1 irrespective
		of the quantity of data actually received before that point.

		@return The length of the string (including the new line), or -1 if a timeout occured. */
		virtual ssize_t ReadLine (std::string &buffer) { return ReadStringUntil (buffer, '\n'); }

		/** @brief Dump data until the specified number of bytes have been read.

 		@return The number of bytes that were skipped, or -1 if a timeout occured. */
		virtual ssize_t Skip (size_t count);

		/** @brief Read and dump data until the specified termination character has been seen @ref
		count times.

		@return The number of bytes that were skipped, or -1 if a timeout occured. */
		virtual ssize_t SkipUntil (uint8_t terminator, unsigned int count);

		/** @brief Get the number of bytes waiting to be read at the port. Returns immediatly.

		@return The number of bytes available. Will not return less than zero. */
		virtual ssize_t BytesAvailable () = 0;

		/** @brief Get the number of bytes waiting after blocking for the timeout.

		Unlike @ref BytesAvailable, this function will wait for the timeout to occur if no data is
		available straight away.

		@return The number of bytes waiting to be read, or -1 if a timeout occured. */
		virtual ssize_t BytesAvailableWait () = 0;

		/** @brief Write data to the port.

		Simply writes @ref count bytes of data from @ref buffer to the port. If the port is
		blocking, this will block until it can write more when the port's output buffer is
		full, or until a timeout occurs. The buffer may become full during the write, in which case
		less than @ref count bytes may be written.

		@return The number of bytes actually written. May be 0 if the port's output buffer is
		already full and a timeout occurs. */
		virtual ssize_t Write (const void * const buffer, size_t count) = 0;

		/** @brief Write all the data to the port.

		Similar to @ref Write, but will keep trying until all data is written to the port rather
		than just writing what it can and returning, even if a timeout occurs.

		@return The number of bytes actually written. */
		virtual ssize_t WriteFull (const void * const buffer, size_t count);

		/** @brief Write a string to the port.

		A convenience function that writes a null-terminated string to the port. Behaves identically
		to @ref Write. The NULL-terminator is *not* written to the port. If you want one, use @ref
		write to send the whole string, or use this function followed by a call to @ref write to
		write the NULL terminator.

		@return The number of bytes actually written. May be 0 if the port's output buffer is
		already full and a timeout occurs. */
		virtual ssize_t WriteString (const char * const buffer);
		virtual ssize_t WriteString (const std::string &buffer)
			{ return WriteString (buffer.c_str ()); }

		/// @brief Flush the port's input and output buffers, discarding all data.
		virtual void Flush () = 0;

		/** @brief Drain the port's output buffers.

		Waits until timeout for the port to finish transmitting data in its output buffer. */
		virtual void Drain () = 0;

		/// @brief Get the status of the port (type, device, etc).
		virtual std::string GetStatus () const;

		// Accessor methods.
		/// @brief Get the port type.
		std::string GetPortType () const        { return _type; }
		/// @brief Set the debug level.
		void SetDebug (int debug)               { _debug = debug; }
		/// @brief Get the debug level.
		int GetDebug () const                   { return _debug; }
		/** @brief Set the timeout value. Set seconds to -1 to disable timeouts and block forever.

		@note On Mac OS X, the timer is reset each time data is received, making the timeout an
		inactivity timer in that there must be no data at all for the length of the timeout for it
		to trigger. This can potentially lead to very long blocking if the sender is sending data
		slightly faster than the timeout. */
		virtual void SetTimeout (Timeout timeout) = 0;
		/// @brief Get the timeout.
		virtual Timeout GetTimeout () const     { return _timeout; }
		/// @brief Get the blocking property of the port. If the timeout is non-zero, the port will
		/// block.
		virtual bool IsBlocking () const        { return (_timeout._sec != 0 ||
                                                          _timeout._usec != 0); }
		/// @brief Set the read permissions of the port.
		virtual void SetCanRead (bool canRead) = 0;
		/// @brief Get the read permissions of the port.
		virtual bool CanRead () const           { return _canRead; }
		/// @brief Set the write permissions of the port.
		virtual void SetCanWrite (bool canWrite) = 0;
		/// @brief Get the write permissions of the port.
		virtual bool CanWrite () const          { return _canWrite; }
		/// @brief Check if the port is open
		virtual bool IsOpen () const = 0;

	protected:
		std::string _type;  // Port type string (e.g. "tcp" or "serial" or "usb")
		unsigned int _debug;
		Timeout _timeout;   // Timeout in milliseconds. Set to zero for non-blocking operation.
		bool _canRead;      // If true, this port can be read from.
		bool _canWrite;     // If true, this port can be written to.
		bool _alwaysOpen;   // If the port should be kept open for the life of the object (including
		                    // reopening it if necessary).

		// Protected constructor to prevent direct creation of this class.
		Port ();
		// Constructor for more-direct creation.
		Port (unsigned int debug, Timeout timeout, bool canRead, bool canWrite, bool alwaysOpen);

		void ProcessOptions (const std::map<std::string, std::string> &options);
		virtual bool ProcessOption (const std::string &option, const std::string &value);
		virtual void CheckPort (bool read) = 0;

	private:
		// Private copy constructor to prevent unintended copying.
		Port (const Port&);
		void operator= (const Port&);
};

} // namespace flexiport

/** @} */

#endif // __PORT_H
