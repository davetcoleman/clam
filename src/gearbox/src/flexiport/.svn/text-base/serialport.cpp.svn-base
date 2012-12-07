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

#include "flexiport.h"
#include "serialport.h"

#if defined (WIN32)
	#define __func__    __FUNCTION__
#else
	#include <sys/ioctl.h>
	#include <termios.h>
	#include <unistd.h>
	#include <errno.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <sstream>
#include <iostream>
using namespace std;

namespace flexiport
{

inline int ErrNo ()
{
#if defined (WIN32)
	return GetLastError ();
#else
	return errno;
#endif
}

inline string StrError (int errNo)
{
#if defined (WIN32)
	LPVOID bufferPointer = NULL;
	FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL,
			errNo, 0, reinterpret_cast<LPTSTR> (&bufferPointer), 0, NULL);
	string result (reinterpret_cast<LPTSTR> (bufferPointer));
	LocalFree (bufferPointer);
	return result;
#else
	return string (strerror (errNo));
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////////////////////////

int BaudToConstant (int baud)
{
    switch (baud)
	{
#if defined (WIN32)
		case 110:
			return CBR_110;
		case 300:
			return CBR_300;
		case 600:
			return CBR_600;
		case 1200:
			return CBR_1200;
		case 2400:
			return CBR_2400;
		case 4800:
			return CBR_4800;
		case 9600:
			return CBR_9600;
		case 14400:
			return CBR_14400;
		case 19200:
			return CBR_19200;
		case 38400:
			return CBR_38400;
		case 57600:
			return CBR_57600;
		case 128000:
			return CBR_128000;
		case 256000:
			return CBR_256000;
#else
		case 0:
			return B0;
		case 50:
			return B50;
		case 75:
			return B75;
		case 110:
			return B110;
		case 134:
			return B134;
		case 150:
			return B150;
		case 200:
			return B200;
		case 300:
			return B300;
		case 600:
			return B600;
		case 1200:
			return B1200;
		case 1800:
			return B1800;
		case 2400:
			return B2400;
		case 4800:
			return B4800;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
#if defined (__linux)
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 500000:
			return B500000;
		case 576000:
			return B576000;
		case 921600:
			return B921600;
		case 1000000:
			return B1000000;
		case 1152000:
			return B1152000;
		case 1500000:
			return B1500000;
		case 2000000:
			return B2000000;
		case 2500000:
			return B2500000;
		case 3000000:
			return B3000000;
		case 3500000:
			return B3500000;
		case 4000000:
			return B4000000;
#endif // __linux
#endif
		default:
			stringstream ss;
			ss << "SerialPort::" << __func__ << "() Invalid baud rate: " << baud;
			throw PortException (ss.str());
	}
}

int ConstantToBaud (int constant)
{
    switch (constant)
	{
#if defined (WIN32)
		case CBR_110:
			return 110;
		case CBR_300:
			return 300;
		case CBR_600:
			return 600;
		case CBR_1200:
			return 1200;
		case CBR_2400:
			return 2400;
		case CBR_4800:
			return 4800;
		case CBR_9600:
			return 9600;
		case CBR_14400:
			return 14400;
		case CBR_19200:
			return 19200;
		case CBR_38400:
			return 38400;
		case CBR_57600:
			return 57600;
		case CBR_128000:
			return 128000;
		case CBR_256000:
			return 256000;
#else
		case B0:
			return 0;
		case B50:
			return 50;
		case B75:
			return 75;
		case B110:
			return 110;
		case B134:
			return 134;
		case B150:
			return 150;
		case B200:
			return 200;
		case B300:
			return 300;
		case B600:
			return 600;
		case B1200:
			return 1200;
		case B1800:
			return 1800;
		case B2400:
			return 2400;
		case B4800:
			return 4800;
		case B9600:
			return 9600;
		case B19200:
			return 19200;
		case B38400:
			return 38400;
		case B57600:
			return 57600;
		case B115200:
			return 115200;
#if defined (__linux)
		case B230400:
			return 230400;
		case B460800:
			return 460800;
		case B500000:
			return 500000;
		case B576000:
			return 576000;
		case B921600:
			return 921600;
		case B1000000:
			return 1000000;
		case B1152000:
			return 1152000;
		case B1500000:
			return 1500000;
		case B2000000:
			return 2000000;
		case B2500000:
			return 2500000;
		case B3000000:
			return 3000000;
		case B3500000:
			return 3500000;
		case B4000000:
			return 4000000;
#endif // __linux
#endif
		default:
			stringstream ss;
			ss << "SerialPort::" << __func__ << "() Invalid baud constant: " << constant;
			throw PortException (ss.str());
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////////////////

SerialPort::SerialPort (map<string, string> options)
	: Port (),
#if defined (WIN32)
	_fd (INVALID_HANDLE_VALUE),
#else
	_fd (-1),
#endif
	_device ("/dev/ttyS0"), _baud (9600), _dataBits (8),
	_stopBits (1), _parity (PAR_NONE), _hwFlowCtrl (false), _open (false)
{
	_type = "serial";
	ProcessOptions (options);

	if (_alwaysOpen)
		Open ();
}

SerialPort::~SerialPort ()
{
	Close ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Port management
////////////////////////////////////////////////////////////////////////////////////////////////////

void SerialPort::Open ()
{
	if (_open)
		throw PortException ("Attempt to open already-opened port.");

	if (_debug >= 1)
		cerr << "SerialPort::" << __func__ << "() Opening serial device " << _device << endl;

#if defined (WIN32)
	DWORD flags = 0;
	if (_canRead)
		flags |= GENERIC_READ;
	if (_canWrite)
		flags |= GENERIC_WRITE;

	// TODO:
	// Because the Win32 serial port API is a piece of crap, we have to use overlapped files to get
	// anywhere near the flexibility we get with the POSIX API (such as decent timeout support, the
	// ability to wait for data to be available without blocking forever, etc). The downside is that
	// this makes the operations that are simple for non-overlapped IO become nasty.
//	if ((_fd = CreateFile (_device.c_str (), flags, 0, 0,
//							OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
//							NULL)) == INVALID_HANDLE_VALUE)
	if ((_fd = CreateFile (_device.c_str (), flags, 0, 0,
							OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,
							NULL)) == INVALID_HANDLE_VALUE)
	{
		stringstream ss;
		ss << "Failed to open device " << _device << " with error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	SetPortTimeout ();
#else
	int flags = 0;
	if (_canRead && _canWrite)
		flags = O_RDWR;
	else if (_canRead)
		flags = O_RDONLY;
	else
		flags = O_WRONLY;

	flags |= O_NOCTTY;

	// For simplicity set the port to non-blocking and use select() to wait for data based on the
	// timeout.
	flags |= O_NONBLOCK;

	if ((_fd = open (_device.c_str (), flags)) < 0)
	{
		stringstream ss;
		ss << "Failed to open device " << _device << " with error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	SetPortSettings ();

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Serial device opened." << endl;
	_open = true;
}

void SerialPort::Close ()
{
	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Closing port" << endl;

#if defined (WIN32)
	if (_fd != INVALID_HANDLE_VALUE)
	{
		CloseHandle (_fd);
		_fd = INVALID_HANDLE_VALUE;
	}
#else
	if (_fd >= 0)
	{
		close (_fd);
		_fd = -1;
	}
#endif
	_open = false;

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Port closed" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t SerialPort::Read (void * const buffer, size_t count)
{
	CheckPort (true);

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Going to read " << count << " bytes" << endl;

#if defined (WIN32)
	DWORD receivedBytes = 0;
	if (!ReadFile (_fd, buffer, count, &receivedBytes, NULL))
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() ReadFile() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	// TODO: How to tell if the port has closed in Win32?
	if (receivedBytes == 0)
	{
		// No data received indicates a timeout in Win32
		return -1;
	}
#else
	ssize_t receivedBytes = 0;

	if (_timeout._sec == -1)
	{
		// Port is blocking, so just read
		receivedBytes = read (_fd, buffer, count);
	}
	else
	{
		// Port is non-blocking, so try to read, see if we get any data (if there is none, this will
		// return immediately, and is much faster than ioctl() and select() calls)
		receivedBytes = read (_fd, buffer, count);
		// Check if that call "timed out"
		if (receivedBytes < 0 && ErrNo () == EAGAIN)
		{
			// No data was available, so wait for data or timeout, then read if data is available
			if (WaitForDataOrTimeout () == TIMED_OUT)
				return -1;
			receivedBytes = read (_fd, buffer, count);
		}
		// If first call doesn't return a timeout, fall through to the result/error checking below
	}

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Read " << receivedBytes << " bytes" << endl;

	if (receivedBytes < 0)
	{
		if (ErrNo () == EAGAIN)
			return -1; // Timed out
		else
		{
			// General error
			stringstream ss;
			ss << "SerialPort::" << __func__ << "() read() error: (" <<
				ErrNo () << ") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}
	else if (receivedBytes == 0)
	{
		// Port has closed, do the same at this end
		if (_debug >= 1)
			cerr << "SerialPort::" << __func__ << "() Port has closed." << endl;
		Close ();
		// Reopen if set to do so
		if (_alwaysOpen)
		{
			if (_debug >= 1)
				cerr << "SerialPort::" << __func__ << "() Trying to reopen." << endl;
			Open ();
		}
		return 0;
	}
#endif

	return receivedBytes;
}

ssize_t SerialPort::ReadFull (void * const buffer, size_t count)
{
	size_t receivedBytes = 0;
	Timeout oldTimeout = _timeout;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "SerialPort::" << __func__ << "() Going to read until have " << count <<
			" bytes" << endl;
	}

	// Set the port to infinite blocking
	SetTimeout (Timeout (-1, 0));
	// Keep calling Read() until count bytes have been received or a timeout
	while (receivedBytes < count)
	{
		ssize_t numReceived = Read (&(reinterpret_cast<uint8_t*> (buffer)[receivedBytes]),
								count - receivedBytes);
		if (numReceived < 0)
		{
			// Timed out (how?!)
			// Restore the timeout
			SetTimeout (oldTimeout);
			// Throw up
			throw PortException (string ("SerialPort::") + __func__ +
					string (" Read() timed out, probably shouldn't happen."));
		}
		else if (numReceived == 0)
		{
			// There's a chance that the port is no longer open
			if (!IsOpen ())
			{
				// Restore the timeout
				SetTimeout (oldTimeout);
				stringstream ss;
				ss << "SerialPort::" << __func__ << "() Port closed while trying to read " <<
					count << " bytes";
				throw PortException (ss.str ());
			}
			// If it is open we can keep going, but with a warning
			if (_debug >= 1)
			{
				cerr << "WARNING: SerialPort::" << __func__ <<
					" Port closed during ReadFull operation; data may be missing/corrupted." <<
					endl;
			}
		}
		// Otherwise got data or not, continue as normal
		receivedBytes += numReceived;
	}

	// Restore the timeout
	SetTimeout (oldTimeout);
	return receivedBytes;
}

ssize_t SerialPort::BytesAvailable ()
{
	ssize_t bytesAvailable = 0;

	CheckPort (true);

#if defined (WIN32)
	COMSTAT comStat;
	DWORD errorType;
	if (!ClearCommError (_fd, &errorType, &comStat))
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() ClearCommError() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	bytesAvailable = comStat.cbInQue;
#else
	if (ioctl (_fd, FIONREAD, &bytesAvailable) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	if (_debug >= 2)
	{
		cerr << "SerialPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available" << endl;
	}
	return bytesAvailable;
}

ssize_t SerialPort::BytesAvailableWait ()
{
	ssize_t bytesAvailable = 0;

	CheckPort (true);

#if defined (WIN32)
	if ((bytesAvailable = BytesAvailable ()) <= 0)
	{
		// TODO: Fix this when doing overlapped mode
		// Sleep for the timeout (ug)
		if (_timeout._sec == -1)
		{
			// Block forever
		}
		else
		{
			// Sleep for the length of the timeout
			Sleep (_timeout._sec * 1000 + _timeout._usec / 1000);
		}
		bytesAvailable = BytesAvailable ();
	}
#else
	// WaitForDataOrTimeout() will do an initial check to see if there is data available immediately
	// and only select() if there isn't
	if (WaitForDataOrTimeout () == TIMED_OUT)
	{
		if (_debug >= 2)
		{
			cerr << "SerialPort::" << __func__ <<
				" Timed out waiting for data to check bytes available" << endl;
		}
		if (IsBlocking ())
			return -1; // Timeout in blocking mode
		else
			return 0; // No data in non-blocking mode
	}

	if (ioctl (_fd, FIONREAD, &bytesAvailable) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	if (_debug >= 2)
	{
		cerr << "SerialPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available after waiting" << endl;
	}
	return bytesAvailable;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t SerialPort::Write (const void * const buffer, size_t count)
{
	CheckPort (false);

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Writing " << count << " bytes" << endl;

#if defined (WIN32)
	DWORD numWritten = 0;
	if (!WriteFile (_fd, buffer, count, &numWritten, NULL))
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() WriteFile() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (numWritten == 0)
	{
		// No data written indicates a timeout in Win32
		return -1;
	}
#else
	ssize_t numWritten = 0;
	if (_timeout._sec != -1)
	{
		if (WaitForWritableOrTimeout () == TIMED_OUT)
		{
			if (_debug >= 2)
				cerr << "SerialPort::" << __func__ << "() Timed out waiting to write" << endl;
			return -1;
		}
	}
	if ((numWritten = write (_fd, buffer, count)) < 0)
	{
		cerr << "numWritten = " << numWritten << " errno = " << errno << endl;
		if (ErrNo () == EAGAIN)
		{
			if (_debug >= 2)
				cerr << "SerialPort::" << __func__ << "() Timed out while in write()" << endl;
			return -1; // Timed out
		}
		else
		{
			// General error
			stringstream ss;
			ss << "SerialPort::" << __func__ << "() write() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}
#endif

	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Wrote " << numWritten << " bytes" << endl;

	return numWritten;
}

void SerialPort::Flush ()
{
#if defined (WIN32)
	if (!PurgeComm (_fd, PURGE_RXCLEAR | PURGE_TXCLEAR))
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() PurgeComm() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#else
	if (tcflush (_fd, TCIOFLUSH) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcflush() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif
}

void SerialPort::Drain ()
{
#if defined (WIN32)
	if (!FlushFileBuffers (_fd))
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() FlushFileBuffers() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#else
	if (tcdrain (_fd) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcdrain() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

std::string SerialPort::GetStatus () const
{
	stringstream status;

	status << "Serial-specific status:" << endl;
	status << "Device: " << _device << endl;
	status << "Baud rate: " << _baud << "\tData bits: " << _dataBits << endl;
	status << "Stop bits: " << _stopBits << "\tParity: ";
	switch (_parity)
	{
		case PAR_NONE:
			status << "None" << endl;
			break;
		case PAR_EVEN:
			status << "Even" << endl;
			break;
		case PAR_ODD:
			status << "Odd" << endl;
			break;
		default:
			throw PortException (string ("SerialPort::") + __func__ +
					string (" Unknown parity setting."));
			break;
	}
	status << "Hardware flow control: " << _hwFlowCtrl << endl;
	status << (_open ? "Port is open" : "Port is closed") << endl;

	return Port::GetStatus () + status.str ();
}

void SerialPort::SetTimeout (Timeout timeout)
{
	_timeout = timeout;
	SetPortTimeout ();
}

void SerialPort::SetCanRead (bool canRead)
{
	if (IsOpen ())
	{
		throw PortException (string ("SerialPort::") + __func__ +
				string (" Cannot change read capability of an open port."));
	}
	_canRead = canRead;
}

void SerialPort::SetCanWrite (bool canWrite)
{
	if (IsOpen ())
	{
		throw PortException (string ("SerialPort::") + __func__ +
				string (" Cannot change write capability of an open port."));
	}
	_canWrite = canWrite;
}

void SerialPort::SetBaudRate (unsigned int baud)
{
#if defined (WIN32)
	DCB dcb = {0};

	if (!GetCommState (_fd, &dcb))
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() GetCommState() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	dcb.BaudRate = BaudToConstant (_baud);

	if (!SetCommState (_fd, &dcb))
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() SetCommState() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#else
	struct termios tio;

	if (tcgetattr (_fd, &tio) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcgetattr() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (cfsetispeed (&tio, BaudToConstant (_baud)) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() cfsetispeed() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	// No, this isn't a repeat of the previous if
	if (cfsetospeed (&tio, BaudToConstant (_baud)) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() cfsetospeed() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (tcsetattr (_fd, TCSAFLUSH, &tio) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcsetattr() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	_baud = baud;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

bool SerialPort::ProcessOption (const std::string &option, const std::string &value)
{
	char c = '\0';

	// Check if the parent class can handle this option
	if (Port::ProcessOption (option, value))
		return true;

	if (option == "device")
	{
		_device = value;
		return true;
	}
	else if (option == "baud")
	{
		istringstream is (value);
		if (!(is >> _baud) || is.get (c) || _baud == 0)
			throw PortException ("Bad baud rate: " + value);
		return true;
	}
	else if (option == "databits")
	{
		istringstream is (value);
		if (!(is >> _dataBits) || is.get (c) || (_dataBits < 5 && _dataBits > 8))
			throw PortException ("Bad data bits value: " + value);
		return true;
	}
	else if (option == "stopbits")
	{
		istringstream is (value);
		if (!(is >> _dataBits) || is.get (c) || (_stopBits != 1 && _stopBits != 2))
			throw PortException ("Bad stop bits value: " + value);
		return true;
	}
	else if (option == "parity")
	{
		if (value == "none")
			_parity = PAR_NONE;
		else if (value == "even")
			_parity = PAR_EVEN;
		else if (value == "odd")
			_parity = PAR_ODD;
		else
			throw PortException ("Bad parity value: " + value);
		return true;
	}
	else if (option == "hwflowctrl")
	{
		_hwFlowCtrl = true;
		return true;
	}

	return false;
}

#if !defined (WIN32)
// Checks if data is available, waiting for the timeout if none is available immediately
SerialPort::WaitStatus SerialPort::WaitForDataOrTimeout ()
{
	// Check if there is data available immediately before spending time on a select() call
	if (BytesAvailable () > 0)
		return DATA_AVAILABLE;

	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_fd, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_fd + 1, &fdSet, NULL, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 3)
			cerr << "SerialPort::" << __func__ << "() Timed out" << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 2)
		cerr << "SerialPort::" << __func__ << "() Found data waiting" << endl;
	return DATA_AVAILABLE;
}

// Checks if the port can be written to, waiting for the timeout if it can't be written immediately
SerialPort::WaitStatus SerialPort::WaitForWritableOrTimeout ()
{
	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_fd, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_fd + 1, NULL, &fdSet, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 3)
			cerr << "SerialPort::" << __func__ << "() Timed out" << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 3)
		cerr << "SerialPort::" << __func__ << "() Found space to write" << endl;
	return CAN_WRITE;
}
#endif // !WIN32

// Check if the port is open and if permissions are set correctly for the desired operation
void SerialPort::CheckPort (bool read)
{
	if (!_open)
		throw PortException ("Port is not open.");

	if (read && !_canRead)
		throw PortException ("Cannot read from write-only port.");

	if (!read && !_canWrite)
		throw PortException ("Cannot write to read-only port.");
}

void SerialPort::SetPortSettings ()
{
#if defined (WIN32)
	DCB dcb = {0};

	if (!GetCommState (_fd, &dcb))
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() GetCommState() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	dcb.fBinary = true;
	// Clear some control line flags we don't want to use
	dcb.fOutxDsrFlow = false;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = false;
	dcb.fOutX = false;
	dcb.fInX = false;
	dcb.fErrorChar = false;
	dcb.fNull = false;

	dcb.ByteSize = _dataBits;

	switch (_parity)
	{
		case PAR_NONE:
			dcb.fParity = false;
			break;
		case PAR_EVEN:
			dcb.fParity = false;
			dcb.Parity = EVENPARITY;
			break;
		case PAR_ODD:
			dcb.fParity = false;
			dcb.Parity = ODDPARITY;
			break;
	}

	if (_stopBits == 2)
		dcb.StopBits = TWOSTOPBITS;
	else
		dcb.StopBits = ONESTOPBIT;

	// Set hardware flow control accordingly
	if (_hwFlowCtrl)
	{
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
		dcb.fOutxCtsFlow = true;
	}
	else
	{
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fOutxCtsFlow = false;
	}

	if (!SetCommState (_fd, &dcb))
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() SetCommState() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#else
	struct termios tio;

	if (tcgetattr (_fd, &tio) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcgetattr() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	// Make it raw first, then configure various options after since cfmakeraw clears some of the
	// flags we may set.
	cfmakeraw (&tio);

	tio.c_cflag &= ~CSIZE;
	switch (_dataBits)
	{
		case 5:
			tio.c_cflag |= CS5;
			break;
		case 6:
			tio.c_cflag |= CS6;
			break;
		case 7:
			tio.c_cflag |= CS7;
			break;
		case 8:
			tio.c_cflag |= CS8;
			break;
		default:
			stringstream ss;
			ss << "Bad data bits value: " << _dataBits;
			throw PortException (ss.str ());
			break;
	}

	if (_stopBits == 2)
		tio.c_cflag |= CSTOPB;
	else
		tio.c_cflag &= ~CSTOPB;

	switch (_parity)
	{
		case PAR_NONE:
			tio.c_cflag &= ~PARENB;
			break;
		case PAR_EVEN:
			tio.c_cflag |= PARENB;
			tio.c_cflag &= ~PARODD;
			break;
		case PAR_ODD:
			tio.c_cflag |= PARENB;
			tio.c_cflag |= PARODD;
			break;
	}

	// Turn the receiver on if read permission is set, off otherwise
	if (_canRead)
		tio.c_cflag |= CREAD;
	else
		tio.c_cflag &= ~CREAD;

	// Ignore the modem control lines
	tio.c_cflag |= CLOCAL;

	// Set hardware flow control accordingly
	if (_hwFlowCtrl)
		tio.c_cflag |= CRTSCTS;
	else
		tio.c_cflag &= ~CRTSCTS;

	if (tcsetattr (_fd, TCSAFLUSH, &tio) < 0)
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() tcsetattr() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	SetBaudRate (_baud);
}

void SerialPort::SetPortTimeout ()
{
#if defined (WIN32)
	COMMTIMEOUTS timeouts;

	if (_timeout._sec == -1)
	{
		// Block forever
		timeouts.ReadIntervalTimeout = 0;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.ReadTotalTimeoutConstant = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
	}
	else if (_timeout._sec == 0 && _timeout._usec == 0)
	{
		// Non-blocking operation
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.ReadTotalTimeoutConstant = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
	}
	else
	{
		// Timeout-based operation
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
		timeouts.ReadTotalTimeoutConstant = _timeout._sec * 1000 + _timeout._usec / 1000;
		timeouts.WriteTotalTimeoutMultiplier = MAXDWORD;
		timeouts.WriteTotalTimeoutConstant = _timeout._sec * 1000 + _timeout._usec / 1000;
	}

	if (!SetCommTimeouts (_fd, &timeouts))
	{
		Close ();
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() SetCommTimeouts() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#else
	int flags;
	if ((flags = fcntl (_fd, F_GETFL)) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() fcntl(F_GETFL) error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (_timeout._sec == -1)
	{
		// Block forever
		flags &= ~O_NONBLOCK;
	}
	else
	{
		// Non-blocking operation
		flags |= O_NONBLOCK;
	}
	if (fcntl (_fd, F_SETFL, flags) < 0)
	{
		stringstream ss;
		ss << "SerialPort::" << __func__ << "() fcntl(F_SETFL) error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif
}

} // namespace flexiport
