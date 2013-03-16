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

#ifndef __SERIALPORT_H
#define __SERIALPORT_H

#include "port.h"

#include <map>
#include <string>
#if defined (WIN32)
	#include <Windows.h>
#endif

/** @ingroup gbx_library_flexiport
@{
*/

namespace flexiport
{

/** @brief Serial implementation of the @ref Port class.

See the @ref Port class documentation for how to use the common API.

@note Under Windows, the timeout resolution is milliseconds, not microseconds. The timeout value
will be rounded to the nearest millisecond when used.

@par Options
 - device <string>
   - Serial device to use.
   - Default: /dev/ttyS0
 - parity none|even|odd
   - Data parity.
   - Default: none
 - baud <integer>
   - Baud rate. Valid values are 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600,
     19200, 34800, 57600, 115200, 230400.
   - Default: 9600
 - databits <integer>
   - Default: 8
 - stopbits 1|2
   - Number of stop bits: 1 or 2.
   - Default: 1
 - hwflowctrl
   - Turn hardware flow control on.
   - Default: off */
class FLEXIPORT_EXPORT SerialPort : public Port
{
	public:
		SerialPort (std::map<std::string, std::string> options);
		~SerialPort ();

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
		/** @brief Get the number of bytes waiting after blocking for the timeout.

		@note Currently this function performs differently under Win32. It will block for the
		entire length of the timeout if data is not immediatly available. This will be fixed in the
		future. */
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
		/// @brief Check if the port is open.
		bool IsOpen () const                        { return _open; }

		/// @brief Change the baud rate.
		void SetBaudRate (unsigned int baud);
		/// @brief Get the current baud rate.
		unsigned int GetBaudRate () const           { return _baud; }

	private:
#if defined (WIN32)
		HANDLE _fd;             // Serial device handle
#else
		int _fd;                // Serial device file descriptor
#endif

		std::string _device;
		unsigned int _baud;
		unsigned int _dataBits;
		unsigned int _stopBits;
		typedef enum {PAR_NONE, PAR_EVEN, PAR_ODD} Parity;
		Parity _parity;
		bool _hwFlowCtrl;
		bool _open;

		void CheckPort (bool read);
		bool ProcessOption (const std::string &option, const std::string &value);

		bool IsDataAvailable ();
#if !defined (WIN32)
		typedef enum {TIMED_OUT, DATA_AVAILABLE, CAN_WRITE} WaitStatus;
		WaitStatus WaitForDataOrTimeout ();
		WaitStatus WaitForWritableOrTimeout ();
#endif
		void SetPortSettings ();
		void SetPortTimeout ();
};

} // namespace flexiport

/** @} */

#endif // __SERIALPORT_H
