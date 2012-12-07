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
#include "udpport.h"

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
	#include <arpa/inet.h>
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

// Yay Beej's!
inline void* GetInAddr (struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET)
		return &(reinterpret_cast<struct sockaddr_in*>(sa)->sin_addr);
	else
		return &(reinterpret_cast<struct sockaddr_in6*>(sa)->sin6_addr);
}

#if defined (WIN32)
	const int ERRNO_EAGAIN = WSAEWOULDBLOCK;
#else
	const int ERRNO_EAGAIN = EAGAIN;
#endif

#if defined (WIN32)
	// Unfortunately, Windows has a fit if these are made class variables due to the inclusion
	// of winsock2.h in udpport.h. No idea why, but it somehow drags in winsock.h, which really
	// doesn't play nice with winsock2.h (despite guards that supposedly prevent it being included
	// at the same time).
	#if defined (FLEXIPORT_HAVE_GETADDRINFO)
		struct sockaddr _destSockAddr;
	#else
		struct sockaddr_in _destSockAddr;
	#endif
#endif

void SetBroadcastFlag (int sock)
{
	int broadcastFlag = 1;
	if (setsockopt (sock, SOL_SOCKET, SO_BROADCAST,
#if defined (WIN32)
					reinterpret_cast<const char*> (&broadcastFlag), sizeof (broadcastFlag)) < 0)
#else
					&broadcastFlag, sizeof (broadcastFlag)) < 0)
#endif
	{
		stringstream ss;
		ss << __func__ << "() setsockopt() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////////////////

UDPPort::UDPPort (map<string, string> options)
#if defined (WIN32)
	: Port (), _sendSock (INVALID_SOCKET), _recvSock (INVALID_SOCKET),
#else
	: Port (), _sendSock (-1), _recvSock (-1),
#endif
	_destIP ("127.0.0.1"), _destPort (20000), _recvIP ("*"), _recvPort (20000),	_open (false)
{
	_type = "udp";
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

UDPPort::~UDPPort ()
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

void UDPPort::Open ()
{
	if (_open)
		throw PortException ("Attempt to open already-opened port.");

    if (_canWrite)
		OpenSender ();
	if (_canRead)
		OpenReceiver ();
	SetSocketBlockingFlag ();

	_open = true;
	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Port is open" << endl;
}

void UDPPort::Close ()
{
	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Closing port" << endl;

	_open = false;
	CloseSender ();
	CloseReceiver ();

	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Port closed" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t UDPPort::Read (void * const buffer, size_t count)
{
	ssize_t receivedBytes = 0;
	struct sockaddr_storage from;
#if defined (WIN32)
	int fromLen = sizeof (from);
#else
	socklen_t fromLen = sizeof (from);
#endif

	CheckPort (true);

	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Going to read " << count << " bytes" << endl;

	if (_timeout._sec == -1)
	{
		// Socket is blocking, so just read
#if defined (WIN32)
		receivedBytes = recvfrom (_recvSock, reinterpret_cast<char*> (buffer), count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#else
		receivedBytes = recvfrom (_recvSock, buffer, count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#endif
	}
	else
	{
		// Socket is non-blocking, so try to read, see if we get any data (if there is none, this
		// will return immediately, and is much faster than ioctl() and select() calls)
#if defined (WIN32)
		receivedBytes = recvfrom (_recvSock, reinterpret_cast<char*> (buffer), count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#else
		receivedBytes = recvfrom (_recvSock, buffer, count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#endif
		// Check if that call "timed out"
		if (receivedBytes < 0 && ErrNo () == ERRNO_EAGAIN)
		{
			// No data was available, so wait for data or timeout, then read if data is available
			if (WaitForDataOrTimeout () == TIMED_OUT)
				return -1;
#if defined (WIN32)
			receivedBytes = recvfrom (_recvSock, reinterpret_cast<char*> (buffer), count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#else
			receivedBytes = recvfrom (_recvSock, buffer, count, 0,
									reinterpret_cast<struct sockaddr*> (&from), &fromLen);
#endif
		}
		// If first call doesn't return a timeout, fall through to the result/error checking below
	}

	if (_debug >= 2)
	{
#if defined (WIN32)
		// This will probably go boom and print a weird source IP if IPv6 is used on Windows,
		// but since there's no inet_ntop() on Windows, we have no choice but to use inet_ntoa().
		cerr << "UDPPort::" << __func__ << "() Read " << receivedBytes << " bytes from " <<
			inet_ntoa (reinterpret_cast<struct sockaddr_in*>(&from)->sin_addr) << endl;
#else
		char s[INET6_ADDRSTRLEN];
		cerr << "UDPPort::" << __func__ << "() Read " << receivedBytes << " bytes from " <<
			inet_ntop (from.ss_family, GetInAddr (reinterpret_cast<struct sockaddr*> (&from)),
						s, sizeof (s))
			<< endl;
#endif
	}

	if (receivedBytes < 0)
	{
		if (ErrNo () == ERRNO_EAGAIN)
			return -1; // Timed out
		else
		{
			// General error
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() recvfrom() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}
	else if (receivedBytes == 0)
	{
		// Peer disconnected cleanly, do the same at this end
		if (_debug >= 1)
			cerr << "UDPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
		Close ();
		if (_alwaysOpen)
		{
			if (_debug >= 1)
				cerr << "UDPPort::" << __func__ << "() Trying to reconnect." << endl;
			Open ();
		}
		return 0;
	}

	return receivedBytes;
}

ssize_t UDPPort::ReadFull (void * const buffer, size_t count)
{
	ssize_t numReceived = 0;
	size_t receivedBytes = 0;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "UDPPort::" << __func__ << "() Going to read until have " <<
			count << " bytes" << endl;
	}

	while (receivedBytes < count)
	{
#if defined (WIN32)
		numReceived = recvfrom (_recvSock, &(reinterpret_cast<char*> (buffer)[receivedBytes]),
				count - receivedBytes, 0, NULL, 0); // No MSG_WAITALL on older versions of visual c, it seems
#else
		numReceived = recvfrom (_recvSock, &(reinterpret_cast<char*> (buffer)[receivedBytes]),
				count - receivedBytes, MSG_WAITALL, NULL, 0);
#endif
		if (_debug >= 2)
			cerr << "UDPPort::" << __func__ << "() Received " << numReceived << " bytes" << endl;
		if (numReceived < 0)
		{
			if (ErrNo () == ERRNO_EAGAIN)
			{
				// Timed out (which probably shouldn't happen)
				throw PortException (string ("UDPPort::") + __func__ +
						string ("() recvfrom() timed out, probably shouldn't happen."));
			}
			else
			{
				// General error
				stringstream ss;
				ss << "UDPPort::" << __func__ << "() recvfrom() error: (" << ErrNo () << ") " <<
					StrError (ErrNo ());
				throw PortException (ss.str ());
			}
		}
		else if (numReceived == 0)
		{
			// Peer disconnected cleanly, do the same at this end
			if (_debug >= 1)
				cerr << "UDPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
			Close ();
			if (_alwaysOpen)
			{
				if (_debug >= 1)
					cerr << "UDPPort::" << __func__ << "() Trying to reconnect." << endl;
				Open ();
				// Can go around again after this - if it doesn't open successfully Open() will throw
			}
			else
			{
				throw PortException (string ("UDPPort::") + __func__ +
						string ("() Port closed during read operation."));
			}
		}
		else
			receivedBytes += numReceived;
	}

	return receivedBytes;
}

ssize_t UDPPort::ReadUntil (void * const buffer, size_t count, uint8_t terminator)
{
	// For a datagram protocol, any read will remove the entire datagram from the socket buffer, no
	// matter how much we actually ask for (although we will still get the amount we asked for in
	// out buffer). This means we cannot read 1 byte at a time, because we would only get the first
	// byte of each datagram. However, since reading any of a datagram removes the whole lot, we
	// can just keep reading as much as possible until we see the terminator somewhere in a received
	// chunk of data.
	// This will be fixed if static buffers are introduced.

	size_t numRead = 0;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "UDPPort::" << __func__ << "() Reading until '" << terminator << "' or " <<
			count << " bytes." << endl;
	}
	while (numRead < count)
	{
		ssize_t result = 0;
		if ((result = Read (reinterpret_cast<void*> (&reinterpret_cast<char*> (buffer)[numRead]), count - numRead)) < 0)
			return -1; // Timeout
		else if (result == 0)
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
				cerr << "UDPPort::" << __func__ << "() Got no data when in blocking mode." << endl;
			return 0;
		}
		else
		{
			// Got data
			if (_debug >= 2)
				cerr << "UDPPort::" << __func__ << "() Read " << result << " bytes." << endl;
			// Go through the data just received and check if any bytes match the terminator
			bool foundTerminator = false;
			// numKeep starts at 1 because we will keep at least one byte if the first is terminal
			size_t numKeep = 1;
			for (size_t ii = numRead; ii < numRead + result; ii++, numKeep++)
			{
				if (reinterpret_cast<uint8_t*>(buffer)[ii] == terminator)
				{
					if (_debug >= 2)
						cerr << "UDPPort::" << __func__ << "() Got terminator character." << endl;
					foundTerminator = true;
					// We don't care about any data passed this point - as far as the application
					// knows, it's as much garbage as real data, so don't waste time clearing it
					break;
				}
			}
			numRead += numKeep;
			if (foundTerminator)
			{
				// Got the terminator so stop reading now
				break;
			}
		}
	}

	return numRead;
}

ssize_t UDPPort::ReadStringUntil (std::string &buffer, char terminator)
{
	throw PortException (string ("UDPPort::") + __func__ +
			string ("() This function does not work for datagram protocols."));
	return 0;
}

ssize_t UDPPort::Skip (size_t count)
{
	throw PortException (string ("UDPPort::") + __func__ +
			string ("() This function does not work for datagram protocols."));
	return 0;
}

ssize_t UDPPort::SkipUntil (uint8_t terminator, unsigned int count)
{
	throw PortException (string ("UDPPort::") + __func__ +
			string ("() This function does not work for datagram protocols."));
	return 0;
}

ssize_t UDPPort::BytesAvailable ()
{
	// TODO:
	// MSG_PEEK is apparently bad on Windows so we should drain what we can into a local buffer
	// instead, then use that first during read calls. See http://support.microsoft.com/kb/192599

	CheckPort (true);

#if defined (WIN32)
	unsigned long bytesAvailable = 0;
	if (ioctlsocket (_recvSock, FIONREAD, &bytesAvailable) < 0)
#else
	ssize_t bytesAvailable = 0;
	if (ioctl (_recvSock, FIONREAD, &bytesAvailable) < 0)
#endif
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 2)
	{
		cerr << "UDPPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available" << endl;
	}
	return bytesAvailable;
}

ssize_t UDPPort::BytesAvailableWait ()
{
	CheckPort (true);

	if (WaitForDataOrTimeout () == TIMED_OUT)
	{
		if (_debug >= 2)
		{
			cerr << "UDPPort::" << __func__ <<
				" Timed out waiting for data to check bytes available" << endl;
		}
		if (IsBlocking ())
			return -1; // Timeout in blocking mode
		else
			return 0; // No data in non-blocking mode
	}

	// TODO:
	// MSG_PEEK is apparently bad on Windows so we should drain what we can into a local buffer
	// instead, then use that first during read calls. See http://support.microsoft.com/kb/192599
#if defined (WIN32)
	unsigned long bytesAvailable = 0;
	if (ioctlsocket (_recvSock, FIONREAD, &bytesAvailable) < 0)
#else
	ssize_t bytesAvailable = 0;
	if (ioctl (_recvSock, FIONREAD, &bytesAvailable) < 0)
#endif
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() ioctl() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 2)
	{
		cerr << "UDPPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available after waiting" << endl;
	}
	return bytesAvailable;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t UDPPort::Write (const void * const buffer, size_t count)
{
	ssize_t numSent = 0;

	CheckPort (false);

	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Writing " << count << " bytes" << endl;
	if (_timeout._sec != -1)
	{
		if (WaitForWritableOrTimeout () == TIMED_OUT)
		{
			if (_debug >= 2)
				cerr << "UDPPort::" << __func__ << "() Timed out waiting to send" << endl;
			return -1;
		}
	}

#if defined (WIN32)
	if ((numSent = sendto (_sendSock, reinterpret_cast<const char*> (buffer), count, 0,
							reinterpret_cast<struct sockaddr*> (&_destSockAddr),
							sizeof (_destSockAddr))) < 0)
#else
	if ((numSent = sendto (_sendSock, buffer, count, 0,
							reinterpret_cast<struct sockaddr*> (&_destSockAddr),
							sizeof (_destSockAddr))) < 0)
#endif
	{
		if (ErrNo () == ERRNO_EAGAIN)
		{
			if (_debug >= 2)
				cerr << "UDPPort::" << __func__ << "() Timed out while in sendto()" << endl;
			return -1; // Timed out
		}
		else
		{
			// General error
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() sendto() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}

	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Wrote " << numSent << " bytes" << endl;

	return numSent;
}

void UDPPort::Flush ()
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
		numRead = recvfrom (_sendSock, dump, 128, 0, NULL, 0);
#else
		numRead = recvfrom (_sendSock, dump, 128, MSG_DONTWAIT, NULL, 0);
#endif
		if (numRead < 0 && ErrNo () != ERRNO_EAGAIN)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() recvfrom() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	} while (numRead > 0);

	// We can't do anything about the write buffers.
}

void UDPPort::Drain ()
{
	// Since we can't force the write buffer to send, we can't do anything here.
	if (_debug >= 1)
		cerr << "UDPPort::" << __func__ << "() Can't drain output buffer of UDP port." << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

std::string UDPPort::GetStatus () const
{
	stringstream status;

	status << "UDP-specific status:" << endl;
	status << "Destination address: " << _destIP << ":" << _destPort << endl;
//	if (_ip == "*")
//	{
//		if (_destAddr == NULL)
//			status << "No auto-configured destination found yet." << endl;
//		else
//			status << "Auto-configured destination: " << endl;
//	}
	status << "Listening address: " << _recvIP << ":" << _recvPort << endl;
	status << (_open ? "Port is open" : "Port is closed") << endl;

	return Port::GetStatus () + status.str ();
}

void UDPPort::SetTimeout (Timeout timeout)
{
	_timeout = timeout;
	SetSocketBlockingFlag ();
}

void UDPPort::SetCanRead (bool canRead)
{
	if (canRead)
		OpenReceiver ();
	else
		CloseReceiver ();
	_canRead = canRead;
}

void UDPPort::SetCanWrite (bool canWrite)
{
	if (canWrite)
		OpenSender ();
	else
		CloseSender ();
	_canWrite = canWrite;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

bool UDPPort::ProcessOption (const std::string &option, const std::string &value)
{
	char c = '\0';

	// Check if the parent class can handle this option
	if (Port::ProcessOption (option, value))
		return true;

	if (option == "dest_ip")
	{
		_destIP = value;
		return true;
	}
	else if (option == "dest_port")
	{
		istringstream is (value);
		if (!(is >> _destPort) || is.get (c) || _destPort == 0)
			throw PortException ("Bad destination port number: " + value);
		return true;
	}
	else if (option == "recv_ip")
	{
		_recvIP = value;
		return true;
	}
	else if (option == "recv_port")
	{
		istringstream is (value);
		if (!(is >> _recvPort) || is.get (c) || _recvPort == 0)
			throw PortException ("Bad receive port number: " + value);
		return true;
	}

	return false;
}

// Open the socket for sending data
void UDPPort::OpenSender ()
{
	CloseSender ();    // To make sure

	// If getaddrinfo() is available, much less stuff needs to be hard-coded or copied around.
#if defined (FLEXIPORT_HAVE_GETADDRINFO)
	struct addrinfo *res = NULL, *goodAI = NULL, hints;

	memset (&hints, 0, sizeof (hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_protocol = IPPROTO_UDP;

	int errorCode;
	ostringstream portSS;
	portSS << _destPort;
	if ((errorCode = getaddrinfo (_destIP.c_str (), portSS.str ().c_str (), &hints, &res)) != 0)
	{
		CloseSender ();
		stringstream ss;
#if defined (WIN32)
		ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
#else
		ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
			gai_strerror (errorCode);
#endif
		throw PortException (ss.str ());
	}

	stringstream ss;
	ss << "UDPPort::" << __func__ << "() socket() errors on all interfaces:";
	for (goodAI = res; goodAI != NULL; goodAI = goodAI->ai_next)
	{
		_sendSock = socket (goodAI->ai_family, goodAI->ai_socktype, goodAI->ai_protocol);
#if defined (WIN32)
		if (_sendSock == INVALID_SOCKET)
#else
		if (_sendSock < 0)
#endif
		{
			ss << " (" << ErrNo () << ") " << StrError (ErrNo ()) << ";";
		}
		else
			break;
	}
	if (goodAI == NULL)
	{
		freeaddrinfo (res);
		throw PortException (ss.str ());
	}

	SetBroadcastFlag (_sendSock);

	if (_debug >= 1)
	{
		cerr << "UDPPort::" << __func__ << "() Socket created, will send to " << _destIP << ":" <<
			_destPort << "." << endl;
	}

	memcpy (&_destSockAddr, goodAI->ai_addr, goodAI->ai_addrlen);
	freeaddrinfo (res);
#else // defined (FLEXIPORT_HAVE_GETADDRINFO)
	// Do it the ugly old way.
	sockaddr_in sockAddr;
	memset (&sockAddr, 0, sizeof (sockAddr));

	_sendSock = socket (AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP);
#if defined (WIN32)
	if (_sendSock == INVALID_SOCKET)
#else
	if (_sendSock < 0)
#endif
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	SetBroadcastFlag (_sendSock);

	struct hostent *hp = NULL;
	if ((hp = gethostbyname (_destIP.c_str ())) == NULL)
	{
		CloseSender ();
		throw PortException (string ("UDPPort::") + __func__ + string (" gethostbyname() error."));
	}
	memcpy (&sockAddr.sin_addr, hp->h_addr, hp->h_length);
	sockAddr.sin_family = hp->h_addrtype;
	sockAddr.sin_port = htons (_destPort);

	if (_debug >= 1)
	{
		cerr << "UDPPort::" << __func__ << "() Socket created, will send to " << _destIP << ":" <<
			_destPort << "." << endl;
	}
	memcpy (&_destSockAddr, &sockAddr, sizeof (sockAddr));
#endif // defined (FLEXIPORT_HAVE_GETADDRINFO)
}

// Close the socket for sending data
void UDPPort::CloseSender ()
{
#if defined (WIN32)
	if (_sendSock != INVALID_SOCKET)
	{
		closesocket (_sendSock);
		_sendSock = INVALID_SOCKET;
	}
#else
	if (_sendSock >= 0)
	{
		close (_sendSock);
		_sendSock = -1;
	}
#endif
}

// Open the socket for receiving data
void UDPPort::OpenReceiver ()
{
	CloseReceiver ();    // To make sure

#if defined (FLEXIPORT_HAVE_GETADDRINFO)
	struct addrinfo *res = NULL, *goodAI = NULL, hints;
	memset (&hints, 0, sizeof (hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE;

	ostringstream portSS;
	portSS << _recvPort;
	int errorCode;
	if (_recvIP == "*")
	{
		// Listen on all interfaces
		if ((errorCode = getaddrinfo (NULL, portSS.str ().c_str (), &hints, &res)) != 0)
		{
			stringstream ss;
#if defined (WIN32)
			ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
#else
			ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
				gai_strerror (errorCode);
#endif
			throw PortException (ss.str ());
		}
	}
	else
	{
		// Listen on the specified interface only
		if ((errorCode = getaddrinfo (_recvIP.c_str (), portSS.str ().c_str (), &hints, &res)) != 0)
		{
			stringstream ss;
#if defined (WIN32)
			ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
#else
			ss << "UDPPort::" << __func__ << "() getaddrinfo() error: (" << errorCode << ") " <<
				gai_strerror (errorCode);
#endif
			throw PortException (ss.str ());
		}
	}

	stringstream ss;
	ss << "UDPPort::" << __func__ << "() Errors on all interfaces:";
	for (goodAI = res; goodAI != NULL; goodAI = goodAI->ai_next)
	{
		_recvSock = socket (goodAI->ai_family, goodAI->ai_socktype, goodAI->ai_protocol);
#if defined (WIN32)
		if (_sendSock == INVALID_SOCKET)
#else
		if (_sendSock < 0)
#endif
		{
			ss << "socket(): (" << ErrNo () << ") " << StrError (ErrNo ()) << ";";
			continue;
		}

		if (bind (_recvSock, goodAI->ai_addr, goodAI->ai_addrlen) < 0)
		{
			CloseReceiver ();
			stringstream ss;
			ss << "bind(): (" << ErrNo () << ") " << StrError (ErrNo ()) << ";";
			continue;
		}

		break;
	}
	if (goodAI == NULL)
	{
		freeaddrinfo (res);
		throw PortException (ss.str ());
	}

	SetBroadcastFlag (_recvSock);

	freeaddrinfo (res);
#else // defined (FLEXIPORT_HAVE_GETADDRINFO)
	char hostName[HOST_NAME_MAX + 1] = {'\0'};
	struct hostent *hp = NULL;

	_recvSock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
#if defined (WIN32)
	if (_recvSock == INVALID_SOCKET)
#else
	if (_recvSock < 0)
#endif
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() socket() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	sockaddr_in sockAddr;
	memset (&sockAddr, 0, sizeof (sockAddr));
	if (_recvIP == "*")
	{
		// Listen on all interfaces
		if (gethostname (hostName, HOST_NAME_MAX) < 0)
		{
			{
				stringstream ss;
				ss << "UDPPort::" << __func__ << "() gethostname() error: (" << ErrNo () << ") " <<
					StrError (ErrNo ());
				throw PortException (ss.str ());
			}
		}
		if ((hp = gethostbyname (hostName)) == NULL)
		{
			throw PortException (string ("UDPPort::") + __func__ +
					string (" gethostbyname() error."));
		}
		sockAddr.sin_family = hp->h_addrtype;
		sockAddr.sin_port = htons (_recvPort);
	}
	else
	{
		// Listen on the specified interface only
		if ((hp = gethostbyname (_recvIP.c_str ())) == NULL)
		{
			throw PortException (string ("UDPPort::") + __func__ +
					string (" gethostbyname() error."));
		}
		sockAddr.sin_family = hp->h_addrtype;
		sockAddr.sin_port = htons (_recvPort);
	}

	if (bind (_recvSock, reinterpret_cast<struct sockaddr*> (&sockAddr), sizeof (sockAddr)) < 0)
	{
		CloseReceiver ();
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() bind() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	SetBroadcastFlag (_recvSock);
#endif // defined (FLEXIPORT_HAVE_GETADDRINFO)

	if (_debug >= 1)
		cerr << "UDPPort::" << __func__ << "() Waiting for data." << endl;
}

// Close the socket for receiving data
void UDPPort::CloseReceiver ()
{
#if defined (WIN32)
	if (_recvSock != INVALID_SOCKET)
	{
		closesocket (_recvSock);
		_recvSock = INVALID_SOCKET;
	}
#else
	if (_recvSock >= 0)
	{
		close (_recvSock);
		_recvSock = -1;
	}
#endif
}

// Checks if data is available, waiting for the timeout if none is available immediatly
UDPPort::WaitStatus UDPPort::WaitForDataOrTimeout ()
{
	if (IsDataAvailable ())
	{
		if (_debug >= 2)
			cerr << "UDPPort::" << __func__ << "() Found data available immediately." << endl;
		return DATA_AVAILABLE;
	}
	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() No data available immediately, will wait." << endl;

	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_recvSock, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_recvSock + 1, &fdSet, NULL, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 2)
			cerr << "UDPPort::" << __func__ << "() Timed out" << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 2)
		cerr << "UDPPort::" << __func__ << "() Found data after waiting" << endl;
	return DATA_AVAILABLE;
}

// Checks if data is available right now
bool UDPPort::IsDataAvailable ()
{
	if (_debug >= 3)
		cerr << "UDPPort::" << __func__ << "() Checking if data is available immediately." << endl;

	// First peek at the buffer to see if there is anything waiting
	char buffer;
	ssize_t receivedBytes = 0;
#if defined (WIN32)
	receivedBytes = recvfrom (_recvSock, &buffer, 1, MSG_PEEK, NULL, 0);
#else
	receivedBytes = recvfrom (_recvSock, reinterpret_cast<void*> (&buffer), 1, MSG_PEEK, NULL, 0);
#endif
	if (receivedBytes > 0)
	{
		if (_debug >= 3)
			cerr << "UDPPort::" << __func__ << "() Found data waiting." << endl;
		return DATA_AVAILABLE;
	}
	else if (receivedBytes < 0)
	{
		if (ErrNo () != ERRNO_EAGAIN)
		{
			// General error
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() recvfrom() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		// Else no data available yet
	}
	else // receivedBytes == 0
	{
		// Peer disconnected cleanly, do the same at this end
		if (_debug >= 1)
			cerr << "UDPPort::" << __func__ << "() Peer disconnected cleanly." << endl;
		Close ();
		if (_alwaysOpen)
		{
			if (_debug >= 1)
				cerr << "UDPPort::" << __func__ << "() Trying to reconnect." << endl;
			Open ();
		}
		// Fall through to return no data
	}
	if (_debug >= 3)
		cerr << "UDPPort::" << __func__ << "() Found no data waiting." << endl;
	return false;
}

// Checks it he port can be written to, waiting for the timeout if it can't be written immediatly
UDPPort::WaitStatus UDPPort::WaitForWritableOrTimeout ()
{
	fd_set fdSet;
	struct timeval tv, *tvPtr = NULL;

	FD_ZERO (&fdSet);
	FD_SET (_sendSock, &fdSet);
	tv.tv_sec = _timeout._sec;
	tv.tv_usec = _timeout._usec;
	if (tv.tv_sec >= 0)
		tvPtr = &tv;

	int result = select (_sendSock + 1, NULL, &fdSet, NULL, tvPtr);

	if (result < 0)
	{
		stringstream ss;
		ss << "UDPPort::" << __func__ << "() select() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	else if (result == 0)
	{
		if (_debug >= 3)
			cerr << "UDPPort::" << __func__ << "() Timed out" << endl;
		// Time out
		return TIMED_OUT;
	}
	if (_debug >= 3)
		cerr << "UDPPort::" << __func__ << "() Found space to write" << endl;
	return CAN_WRITE;
}

// Check if the port is open and if permissions are set correctly for the desired operation
void UDPPort::CheckPort (bool read)
{
	if (!_open)
		throw PortException ("Port is not open.");

	if (read && !_canRead)
		throw PortException ("Cannot read from write-only port.");

	if (!read && !_canWrite)
		throw PortException ("Cannot write to read-only port.");
}

void UDPPort::SetSocketBlockingFlag ()
{
	if (_timeout._sec == -1)
	{
		// Disable the non-blocking flag of the socket
#if defined (WIN32)
		unsigned long setting = 0;
		// Receive socket
		if (ioctlsocket (_recvSock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() ioctlsocket(_recvSock) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}

		// Send socket
		if (ioctlsocket (_sendSock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() ioctlsocket(_sendSock) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#else
		int flags;

		// Receive socket
		if ((flags = fcntl (_recvSock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_recvSock, F_GETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags &= ~O_NONBLOCK;
		if (fcntl (_recvSock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_recvSock, F_SETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}

		// Send socket
		if ((flags = fcntl (_sendSock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_sendSock, F_GETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags &= ~O_NONBLOCK;
		if (fcntl (_sendSock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_sendSock, F_SETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#endif
	}
	else
	{
		// Set the socket to non-blocking, and we'll manage timeouts ourselves via select()
#if defined (WIN32)
		unsigned long setting = 1;
		// Receive socket
		if (ioctlsocket (_recvSock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() ioctlsocket(_recvSock) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}

		// Send socket
		if (ioctlsocket (_sendSock, FIONBIO, &setting) == SOCKET_ERROR)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() ioctlsocket(_sendSock) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#else
		int flags;

		// Receive socket
		if ((flags = fcntl (_recvSock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(F_GETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags |= O_NONBLOCK;
		if (fcntl (_recvSock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(F_SETFL) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}

		// Send socket
		if ((flags = fcntl (_sendSock, F_GETFL)) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_sendSock, F_GETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		flags |= O_NONBLOCK;
		if (fcntl (_sendSock, F_SETFL, flags) < 0)
		{
			stringstream ss;
			ss << "UDPPort::" << __func__ << "() fcntl(_sendSock, F_SETFL) error: (" << ErrNo () <<
				") " << StrError (ErrNo ());
			throw PortException (ss.str ());
		}
#endif
	}
}

} // namespace flexiport
