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

#ifndef __TCPPORT_H
#define __TCPPORT_H

#include "port.h"

#include <map>
#include <string>

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/** @brief TCP implementation of the @ref Port class.

See the @ref Port class documentation for how to use the common API.

@par Options
 - ip <string>
   - IP address to connect to. If listen is true, set to "*" to listen on any interface.
   - Default: 127.0.0.1
 - port <integer>
   - TCP port to connect to/listen on.
   - Default: 20000
 - listen
   - Listen on the specified port rather than connecting to it. Other network applications can
     connect and send data, which will become available as normal.
   - Default: off */
class FLEXIPORT_EXPORT TCPPort : public Port
{
	public:
		TCPPort (std::map<std::string, std::string> options);
		~TCPPort ();

		/** @brief Open the port.

		For a listening port, this will call accept() and therefore cause the calling process to
		block until an incoming connection. */
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
		bool IsOpen () const                        { return _open; }

	private:
		int _sock;          // Socket connected to wherever the data is coming from.
		int _listenSock;    // Socket to listen on when in listen mode.

		std::string _ip;
		unsigned int _port;
		bool _isListener;   // True if this port should listen instead of actively connecting.
		bool _open;

		void CheckPort (bool read);

		bool ProcessOption (const std::string &option, const std::string &value);

		void Connect ();
		void WaitForConnection ();
		typedef enum {TIMED_OUT, DATA_AVAILABLE, CAN_WRITE} WaitStatus;
		WaitStatus WaitForDataOrTimeout ();
		bool IsDataAvailable ();
		WaitStatus WaitForWritableOrTimeout ();
		void SetSocketBlockingFlag ();
};

} // namespace flexiport

/** @} */

#endif // __TCPPORT_H
