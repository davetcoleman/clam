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
#include "tcpport.h"
#include "flexiport_config.h"

#if defined (FLEXIPORT_HAVE_GETADDRINFO)
	#include <sys/socket.h>
	#include <netdb.h>
#endif
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <sstream>
#include <iostream>
using namespace std;

#if defined (WIN32)
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#define __func__        __FUNCTION__
#else
	#include <unistd.h>
	#include <errno.h>
	#include <sys/socket.h>
	#include <sys/ioctl.h>
	#include <netdb.h>
#endif

#if !defined (HOST_NAME_MAX)
	#define HOST_NAME_MAX   256
#endif

namespace flexiport
{

inline int ErrNo ()
{
#if defined (WIN32)
	return WSAGetLastError ();
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

#if defined (WIN32)
	const int ERRNO_EAGAIN = WSAEWOULDBLOCK;
#else
	const int ERRNO_EAGAIN = EAGAIN;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////////////////

TCPPort::TCPPort (map<string, string> options)
#if defined (WIN32)
	: Port (), _sock (INVALID_SOCKET), _listenSock (INVALID_SOCKET),
#else
	: Port (), _sock (-1), _listenSock (-1),
#endif
	_ip ("127.0.0.1"), _port (20000), _isListener (false), _open (false)
{
	_type = "tcp";
	ProcessOptions (options);

#if defined (WIN32)
	// First instance, initialise Windows sockets API
	WSADATA info;
	int result;
	if ((result = WSAStartup (MAKEWORD (2, 2), &info)) != 0)
	{
		stringstream ss;
		ss << "Failed to initialise Windows sockets API with error " << result;
		throw PortException (ss.str ());
	}
#endif

	if (_alwaysOpen)
		Open ();
}

TCPPort::~TCPPort ()
{
	Close ();

#if defined (WIN32)
	// Clean up the Windows sockets API
	if (WSACleanup () != 0)
	{
		stringstream ss;
		ss << "Failed to clean up Windows sockets API with error " << WSAGetLastError ();
		throw PortException (ss.str ());
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Port management
////////////////////////////////////////////////////////////////////////////////////////////////////

void TCPPort::Open ()
{
	if (_open)
		throw PortException ("Attempt to open already-opened port.");

	if (_isListener)
	{
		// Wait for a connection on the given address
		if (_debug >= 1)
			cerr << "TCPPort::" << __func__ << "() Listening" << endl;

		WaitForConnection ();
	}
	else
	{
		// Connect to the given address
		if (_debug >= 1)
			cerr << "TCPPort::" << __func__ << "() Connecting" << endl;

		Connect ();
	}
	SetSocketBlockingFlag ();
	_open = true;
	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Port is open" << endl;
}

void TCPPort::Close ()
{
	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Closing port" << endl;

	_open = false;
#if defined (WIN32)
	if (_sock != INVALID_SOCKET)
	{
		closesocket (_sock);
		_sock = INVALID_SOCKET;
	}
	if (_listenSock != INVALID_SOCKET)
	{
		closesocket (_listenSock);
		_listenSock = INVALID_SOCKET;
	}
#else
	if (_sock >= 0)
	{
		close (_sock);
		_sock = -1;
	}
	if (_listenSock >= 0)
	{
		close (_listenSock);
		_listenSock = -1;
	}
#endif

	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Port closed" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t TCPPort::Read (void * const buffer, size_t count)
{
	ssize_t receivedBytes = 0;

	CheckPort (true);

	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Going to read " << count << " bytes" << endl;

	if (_timeout._sec == -1)
	{
		// Socket is blocking, so just read
#if defined (WIN32)
		receivedBytes = recv (_sock, reinterpret_cast<char*> (buffer), count, 0);
#else
		receivedBytes = recv (_sock, buffer, count, 0);
#endif
	}
	else
	{
		// Socket is non-blocking, so try to read, see if we get any data (if there is none, this
		// will return immediately, and is much faster than ioctl() and select() calls)
#if defined (WIN32)
		receivedBytes = recv (_sock, reinterpret_cast<char*> (buffer), count, 0);
#else
		receivedBytes = recv (_sock, buffer, count, 0);
#endif
		// Check if that call "timed out"
		if (receivedBytes < 0 && ErrNo () == ERRNO_EAGAIN)
		{
			// No data was available, so wait for data or timeout, then read if data is available
			if (WaitForDataOrTimeout () == TIMED_OUT)
				return -1;
#if defined (WIN32)
			receivedBytes = recv (_sock, reinterpret_cast<char*> (buffer), count, 0);
#else
			receivedBytes = recv (_sock, buffer, count, 0);
#endif
		}
		// If first call doesn't return a timeout, fall through to the result/error checking below
	}

	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Read " << receivedBytes << " bytes" << endl;

	if (receivedBytes < 0)
	{
		if (ErrNo () == ERRNO_EAGAIN)
			return -1; // Timed out
		else
		{
			// General error
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() recv() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}
	else if (receivedBytes == 0)
	{
		// Peer disconnected cleanly, do the same at this end
		if (_debug >= 1)
			cerr << "TCPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
		Close ();
		if (_alwaysOpen)
		{
			if (_debug >= 1)
				cerr << "TCPPort::" << __func__ << "() Trying to reconnect." << endl;
			Open ();
		}
		return 0;
	}

	return receivedBytes;
}

ssize_t TCPPort::ReadFull (void * const buffer, size_t count)
{
	ssize_t numReceived = 0;
	size_t receivedBytes = 0;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "TCPPort::" << __func__ << "() Going to read until have " <<
			count << " bytes" << endl;
	}

	while (receivedBytes < count)
	{
#if defined (WIN32)
		numReceived = recv (_sock, &(reinterpret_cast<char*> (buffer)[receivedBytes]),
				count - receivedBytes, 0); // No MSG_WAITALL on older versions of visual c, it seems
#else
		numReceived = recv (_sock, &(reinterpret_cast<char*> (buffer)[receivedBytes]),
				count - receivedBytes, MSG_WAITALL);
#endif
		if (_debug >= 2)
			cerr << "TCPPort::" << __func__ << "() Received " << numReceived << " bytes" << endl;
		if (numReceived < 0)
		{
			if (ErrNo () == ERRNO_EAGAIN)
			{
				// Timed out (which probably shouldn't happen)
				throw PortException (string ("TCPPort::") + __func__ +
						string ("() recv() timed out, probably shouldn't happen."));
			}
			else
			{
				// General error
				stringstream ss;
				ss << "TCPPort::" << __func__ << "() recv() error: (" << ErrNo () << ") " <<
					StrError (ErrNo ());
				throw PortException (ss.str ());
			}
		}
		else if (numReceived == 0)
		{
			// Peer disconnected cleanly, do the same at this end
			if (_debug >= 1)
				cerr << "TCPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
			Close ();
			if (_alwaysOpen)
			{
				if (_debug >= 1)
					cerr << "TCPPort::" << __func__ << "() Trying to reconnect." << endl;
				Open ();
				// Can go around again after this - if it doesn't open successfully Open() will throw
			}
			else
			{
				throw PortException (string ("TCPPort::") + __func__ +
						string ("() Port closed during read operation."));
			}
		}
		else
			receivedBytes += numReceived;
	}

	return receivedBytes;
}

ssize_t TCPPort::BytesAvailable ()
{
	// TODO:
	// MSG_PEEK is apparently bad on Windows so we should drain what we can into a local buffer
	// instead, then use that first during read calls. See http://support.microsoft.com/kb/192599.
	// Unless the buffer is static in size this will affect performance and make real-time not work.

	CheckPort (true);

#if defined (WIN32)
	unsigned long bytesAvailable = 0;
	if (ioctlsocket (_sock, FIONREAD, &bytesAvailable) < 0)
#else
	ssize_t bytesAvailable = 0;
	if (ioctl (_sock, FIONREAD, &bytesAvailable) < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 2)
	{
		cerr << "TCPPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available" << endl;
	}
	return bytesAvailable;
}

ssize_t TCPPort::BytesAvailableWait ()
{
	CheckPort (true);

	if (WaitForDataOrTimeout () == TIMED_OUT)
	{
		if (_debug >= 2)
		{
			cerr << "TCPPort::" << __func__ <<
				" Timed out waiting for data to check bytes available" << endl;
		}
		if (IsBlocking ())
			return -1; // Timeout in blocking mode
		else
			return 0; // No data in non-blocking mode
	}

	// TODO:
	// MSG_PEEK is apparently bad on Windows so we should drain what we can into a local buffer
	// instead, then use that first during read calls. See http://support.microsoft.com/kb/192599.
	// Unless the buffer is static in size this will affect performance and make real-time not work.
#if defined (WIN32)
	unsigned long bytesAvailable = 0;
	if (ioctlsocket (_sock, FIONREAD, &bytesAvailable) < 0)
#else
	ssize_t bytesAvailable = 0;
	if (ioctl (_sock, FIONREAD, &bytesAvailable) < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 2)
	{
		cerr << "TCPPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available after waiting" << endl;
	}
	return bytesAvailable;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t TCPPort::Write (const void * const buffer, size_t count)
{
	ssize_t numSent = 0;

	CheckPort (false);

	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Writing " << count << " bytes" << endl;
	if (_timeout._sec != -1)
	{
		if (WaitForWritableOrTimeout () == TIMED_OUT)
		{
			if (_debug >= 2)
				cerr << "TCPPort::" << __func__ << "() Timed out waiting to send" << endl;
			return -1;
		}
	}
#if defined (WIN32)
	if ((numSent = send (_sock, reinterpret_cast<const char*> (buffer), count, 0)) < 0)
#else
	if ((numSent = send (_sock, buffer, count, 0)) < 0)
#endif
	{
		if (ErrNo () == ERRNO_EAGAIN)
		{
			if (_debug >= 2)
				cerr << "TCPPort::" << __func__ << "() Timed out while in send()" << endl;
			return -1; // Timed out
		}
		else
		{
			// General error
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() send() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}

	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Wrote " << numSent << " bytes" << endl;

	return numSent;
}

void TCPPort::Flush ()
{
	int numRead = 0;
	char dump[128];

	// Read data out of the socket into a dump until there's nothing left to read.
	// Use MSG_DONTWAIT to avoid the timeout if one is set on Linux.
	// It would be nice to use MSG_DONTWAIT on Windows, but MS didn't see fit to include that in
	// their cramming of BSD sockets into Windows. Instead, check if data is available before
	// calling recv.
	do
	{
#if defined (WIN32)
		if (!IsDataAvailable ())
			break;
		numRead = recv (_sock, dump, 128, 0);
#else
		numRead = recv (_sock, dump, 128, MSG_DONTWAIT);
#endif
		if (numRead < 0 && ErrNo () != ERRNO_EAGAIN)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() recv() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	} while (numRead > 0);

	// We can't do anything about the write buffers.
}

void TCPPort::Drain ()
{
	// Since we can't force the write buffer to send, we can't do anything here.
	if (_debug >= 1)
		cerr << "TCPPort::" << __func__ << "() Can't drain output buffer of TCP port." << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

std::string TCPPort::GetStatus () const
{
	stringstream status;

	status << "TCP-specific status:" << endl;
	status << "Address: " << _ip << ":" << _port << endl;
	status << (_isListener ? "Is a listener" : "Will actively connect") << endl;
	status << (_open ? "Port is open" : "Port is closed") << endl;

	return Port::GetStatus () + status.str ();
}

void TCPPort::SetTimeout (Timeout timeout)
{
	_timeout = timeout;
	SetSocketBlockingFlag ();
}

void TCPPort::SetCanRead (bool canRead)
{
	_canRead = canRead;
}

void TCPPort::SetCanWrite (bool canWrite)
{
	_canWrite = canWrite;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

bool TCPPort::ProcessOption (const std::string &option, const std::string &value)
{
	char c = '\0';

	// Check if the parent class can handle this option
	if (Port::ProcessOption (option, value))
		return true;

	if (option == "ip")
	{
		_ip = value;
		return true;
	}
	else if (option == "port")
	{
		istringstream is (value);
		if (!(is >> _port) || is.get (c) || _port == 0)
			throw PortException ("Bad port number: " + value);
		return true;
	}
	else if (option == "listen")
	{
		_isListener = true;
		return true;
	}

	return false;
}

// Connect to a remote server: used when not in listen mode
void TCPPort::Connect ()
{
	Close ();    // To make sure

	// If getaddrinfo() is available, much less stuff needs to be hard-coded or copied around.
#if defined (FLEXIPORT_HAVE_GETADDRINFO)
	struct addrinfo *res = NULL, hints;

	memset (&hints, 0, sizeof (hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	int errorCode;
	ostringstream portSS;
	portSS << _port;
	if ((errorCode = getaddrinfo (_ip.c_str (), portSS.str ().c_str (), &hints, &res)) != 0)
	{
		Close ();
		stringstream ss;
#if defined (WIN32)
		ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
#else
		ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
			gai_strerror (errorCode);
#endif
		throw PortException (ss.str ());
	}

	_sock = socket (res->ai_family, res->ai_socktype, res->ai_protocol);
#if defined (WIN32)
	if (_sock == INVALID_SOCKET)
#else
	if (_sock < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 1)
		cerr << "TCPPort::" << __func__ << "() Connecting to " << _ip << ":" << _port << "." << endl;
	if (connect (_sock, res->ai_addr, res->ai_addrlen) < 0)
	{
		Close ();
		stringstream ss;
		ss << "Failed to connect to " << _ip << ": (" << ErrNo () << ") " << StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	freeaddrinfo (res);
#else // defined (FLEXIPORT_HAVE_GETADDRINFO)
	// Do it the ugly old way.
	sockaddr_in sockAddr;
	memset (&sockAddr, 0, sizeof (sockAddr));

	_sock = socket (PF_INET, SOCK_STREAM, 0);
#if defined (WIN32)
	if (_sock == INVALID_SOCKET)
#else
	if (_sock < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	struct hostent *hp = NULL;
	if ((hp = gethostbyname (_ip.c_str ())) == NULL)
	{
		Close ();
		throw PortException (string ("TCPPort::") + __func__ + string (" gethostbyname() error."));
	}
	memcpy (&sockAddr.sin_addr, hp->h_addr, hp->h_length);
	sockAddr.sin_family = hp->h_addrtype;
	sockAddr.sin_port = htons (_port);

	if (_debug >= 1)
		cerr << "TCPPort::" << __func__ << "() Connecting to " << _ip << ":" << _port << "." << endl;
	if (connect (_sock, reinterpret_cast<struct sockaddr*> (&sockAddr), sizeof (sockAddr)) < 0)
	{
		Close ();
		stringstream ss;
		ss << "Failed to connect to " << _ip << ": (" << ErrNo () << ") " << StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif // defined (FLEXIPORT_HAVE_GETADDRINFO)
}

// Wait for a connection: used in listen mode
void TCPPort::WaitForConnection ()
{
	Close ();    // To make sure

#if defined (FLEXIPORT_HAVE_GETADDRINFO)
	struct addrinfo *res = NULL, hints;
	memset (&hints, 0, sizeof (hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	ostringstream portSS;
	portSS << _port;
	int errorCode;
	if (_ip == "*")
	{
		// Listen on all interfaces
		if ((errorCode = getaddrinfo (NULL, portSS.str ().c_str (), &hints, &res)) != 0)
		{
			stringstream ss;
#if defined (WIN32)
			ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
#else
			ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
				gai_strerror (errorCode);
#endif
			throw PortException (ss.str ());
		}
	}
	else
	{
		// Listen on the specified interface only
		if ((errorCode = getaddrinfo (_ip.c_str (), portSS.str ().c_str (), &hints, &res)) != 0)
		{
			stringstream ss;
#if defined (WIN32)
			ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
#else
			ss << "TCPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
				gai_strerror (errorCode);
#endif
			throw PortException (ss.str ());
		}
	}

	_listenSock = socket (res->ai_family, res->ai_socktype, res->ai_protocol);
#if defined (WIN32)
	if (_listenSock == INVALID_SOCKET)
#else
	if (_listenSock < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (bind (_listenSock, res->ai_addr, res->ai_addrlen) < 0)
	{
		Close ();
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() bind() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	freeaddrinfo (res);
#else // defined (FLEXIPORT_HAVE_GETADDRINFO)
	char hostName[HOST_NAME_MAX + 1] = {'\0'};
	struct hostent *hp = NULL;

	_listenSock = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP);
#if defined (WIN32)
	if (_listenSock == INVALID_SOCKET)
#else
	if (_listenSock < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	sockaddr_in sockAddr;
	memset (&sockAddr, 0, sizeof (sockAddr));
	if (_ip == "*")
	{
		// Listen on all interfaces
		if (gethostname (hostName, HOST_NAME_MAX) < 0)
		{
			{
				stringstream ss;
				ss << "TCPPort::" << __func__ << "() gethostname() error: (" << ErrNo () << ") " <<
					StrError (ErrNo ());
				throw PortException (ss.str ());
			}
		}
		if ((hp = gethostbyname (hostName)) == NULL)
		{
			throw PortException (string ("TCPPort::") + __func__ +
					string (" gethostbyname() error."));
		}
		sockAddr.sin_family = hp->h_addrtype;
		sockAddr.sin_port = htons (_port);
	}
	else
	{
		// Listen on the specified interface only
		if ((hp = gethostbyname (_ip.c_str ())) == NULL)
		{
			throw PortException (string ("TCPPort::") + __func__ +
					string (" gethostbyname() error."));
		}
		sockAddr.sin_family = hp->h_addrtype;
		sockAddr.sin_port = htons (_port);
	}

	if (bind (_listenSock, reinterpret_cast<struct sockaddr*> (&sockAddr), sizeof (sockAddr)) < 0)
	{
		Close ();
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() bind() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif // defined (FLEXIPORT_HAVE_GETADDRINFO)

	// All the same from this point on
	listen (_listenSock, 1);

	if (_debug >= 1)
		cerr << "TCPPort::" << __func__ << "() Waiting for a connection." << endl;
	_sock = accept (_listenSock, NULL, NULL);
#if defined (WIN32)
	if (_sock == INVALID_SOCKET)
#else
	if (_sock < 0)
#endif
	{
		Close ();
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() accept() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	// Done with the listening socket so close it
#if defined (WIN32)
	if (closesocket (_listenSock) < 0)
#else
	if (close (_listenSock) < 0)
#endif
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() close(_listenSock) error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#if defined (WIN32)
	_listenSock = INVALID_SOCKET;
#else
	_listenSock = -1;
#endif
}

// Checks if data is available, waiting for the timeout if none is available immediatly
TCPPort::WaitStatus TCPPort::WaitForDataOrTimeout ()
{
	if (IsDataAvailable ())
	{
		if (_debug >= 2)
			cerr << "TCPPort::" << __func__ << "() Found data available immediately." << endl;
		return DATA_AVAILABLE;
	}
	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() No data available immediately, will wait." << endl;

	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_sock, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_sock + 1, &fdSet, NULL, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 2)
			cerr << "TCPPort::" << __func__ << "() Timed out." << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 2)
		cerr << "TCPPort::" << __func__ << "() Found data waiting." << endl;
	return DATA_AVAILABLE;
}

// Checks if data is available right now
bool TCPPort::IsDataAvailable ()
{
	if (_debug >= 3)
		cerr << "TCPPort::" << __func__ << "() Checking if data is available immediately." << endl;

	// First peek at the buffer to see if there is anything waiting. For an infinite timeout,
	// this will block indefinitely until we get data.
	char buffer;
	ssize_t receivedBytes = 0;
#if defined (WIN32)
	receivedBytes = recv (_sock, &buffer, 1, MSG_PEEK);
#else
	receivedBytes = recv (_sock, reinterpret_cast<void*> (&buffer), 1, MSG_PEEK);
#endif
	if (receivedBytes > 0)
	{
		if (_debug >= 3)
			cerr << "TCPPort::" << __func__ << "() Found data waiting." << endl;
		return true;
	}
	else if (receivedBytes < 0)
	{
		if (ErrNo () != ERRNO_EAGAIN)
		{
			// General error
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() recv() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		// Else no data available yet
	}
	else // receivedBytes == 0
	{
		// Peer disconnected cleanly, do the same at this end
		if (_debug >= 1)
			cerr << "TCPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
		Close ();
		if (_alwaysOpen)
		{
			if (_debug >= 1)
				cerr << "TCPPort::" << __func__ << "() Trying to reconnect." << endl;
			Open ();
		}
		// Fall through to return no data
	}
	if (_debug >= 3)
		cerr << "TCPPort::" << __func__ << "() Found no data waiting." << endl;
	return false;
}

// Checks it he port can be written to, waiting for the timeout if it can't be written immediatly
TCPPort::WaitStatus TCPPort::WaitForWritableOrTimeout ()
{
	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_sock, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_sock + 1, NULL, &fdSet, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "TCPPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 3)
			cerr << "TCPPort::" << __func__ << "() Timed out" << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 3)
		cerr << "TCPPort::" << __func__ << "() Found space to write" << endl;
	return CAN_WRITE;
}

// Check if the port is open and if permissions are set correctly for the desired operation
void TCPPort::CheckPort (bool read)
{
	if (!_open)
		throw PortException ("Port is not open.");

	if (read && !_canRead)
		throw PortException ("Cannot read from write-only port.");

	if (!read && !_canWrite)
		throw PortException ("Cannot write to read-only port.");
}

void TCPPort::SetSocketBlockingFlag ()
{
	if (_timeout._sec == -1)
	{
		// Disable the non-blocking flag of the socket
#if defined (WIN32)
		unsigned long setting = 0;
		if (ioctlsocket (_sock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() ioctlsocket error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#else
		int flags;
		if ((flags = fcntl (_sock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() fcntl(F_GETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags &= ~O_NONBLOCK;
		if (fcntl (_sock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() fcntl(F_SETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#endif
	}
	else
	{
		// Set the socket to non-blocking, and we'll manage timeouts ourselves via select()
#if defined (WIN32)
		unsigned long setting = 1;
		if (ioctlsocket (_sock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() ioctlsocket error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#else
		int flags;
		if ((flags = fcntl (_sock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() fcntl(F_GETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags |= O_NONBLOCK;
		if (fcntl (_sock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "TCPPort::" << __func__ << "() fcntl(F_SETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#endif
	}
}

} // namespace flexiport
