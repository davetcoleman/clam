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

#ifndef __UDPPORT_H
#define __UDPPORT_H

#include "port.h"
#include "flexiport_config.h"

#include <map>
#include <string>
#if !defined (WIN32)
	#include <netinet/in.h>
#endif

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/** @brief UDP implementation of the @ref Port class. This class provides UDP communication between
two known end points. It cannot send to any address other than the configured address.

See the @ref Port class documentation for how to use the common API. Note that some parts of the API
do not apply due to the nature of the datagram-oriented protocol. Because each datagram is
individual and no merging is typically performed between datagrams, several flexiport functions do
not work (they were designed for stream-oriented communications). These are @ref ReadStringUntil,
@ref ReadLine (std::string version), @ref Skip, and @ref SkipUntil. This will be (hopefully) be
fixed soon.

TODO: Add support for configuring the destination address based on the first data received, to allow
destination auto-configuration.
TODO: Add an option to turn buffering on, making the UDP protocol function like a stream-based
protocol and so enabling the use of those parts of the API that do not function correctly for a
datagram-based protocol. Alternatively, fix the functions that don't work yet so they read more than
a byte at a time.

@par Options
 - dest_ip <string>
   - IP address to send data to.
   - Default: 127.0.0.1
 - dest_port <integer>
   - UDP port to send data to.
   - Default: 20000
 - recv_ip <string>
   - IP address to receive data on. Set to "*" for receiving on any interface.
   - Default: *
 - recv_port <integer>
   - UDP port to receive data on.
   - Default: 20000 */
class FLEXIPORT_EXPORT UDPPort : public Port
{
	public:
		UDPPort (std::map<std::string, std::string> options);
		~UDPPort ();

		/** @brief Open the port.

		This will create a listening socket and a sending socket. */
		void Open ();
		/// @brief Close the port.
		void Close ();
		/// @brief Read from the port.
		ssize_t Read (void * const buffer, size_t count);
		/// @brief Read the requested quantity of data from the port.
		ssize_t ReadFull (void * const buffer, size_t count);
		/// @brief Read data until a specified termination byte is received.
		ssize_t ReadUntil (void * const buffer, size_t count, uint8_t terminator);
		/// @brief Read a string until the specified termination character is received.
		ssize_t ReadStringUntil (std::string &buffer, char terminator);
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
		bool IsOpen () const                        { return _open; }

	private:
#if !defined (WIN32)
	#if defined (FLEXIPORT_HAVE_GETADDRINFO)
		struct sockaddr _destSockAddr;
	#else
		struct sockaddr_in _destSockAddr;
	#endif
#endif // !defined (WIN32)
		int _sendSock;      // Socket to send data from.
		int _recvSock;      // Socket to receive data on.

		std::string _destIP;
		unsigned int _destPort;
		std::string _recvIP;
		unsigned int _recvPort;
		bool _open;

		void CheckPort (bool read);

		bool ProcessOption (const std::string &option, const std::string &value);

		void OpenSender ();
		void CloseSender ();
		void OpenReceiver ();
		void CloseReceiver ();
		typedef enum {TIMED_OUT, DATA_AVAILABLE, CAN_WRITE} WaitStatus;
		WaitStatus WaitForDataOrTimeout ();
		bool IsDataAvailable ();
		WaitStatus WaitForWritableOrTimeout ();
		void SetSocketBlockingFlag ();
};

} // namespace flexiport

/** @} */

#endif // __UDPPORT_H
