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
#include "logreaderport.h"
#include "logfile.h"

#include <sstream>
#include <iostream>
using namespace std;

#if defined (WIN32)
	#define __func__        __FUNCTION__
#endif

namespace flexiport
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/desctructor
////////////////////////////////////////////////////////////////////////////////////////////////////

LogReaderPort::LogReaderPort (map<string, string> options)
	: Port (), _logFileName ("port.log"), _strictness (0),
	_jitter (100), _ignoreTimes (false), _open (false)
{
	_type = "logreader";
	ProcessOptions (options);

	// Initialise the log file
	_logFile = new LogFile (_debug);
	_logFile->Open (_logFileName, true, _ignoreTimes);

	if (_alwaysOpen)
		Open ();
}

LogReaderPort::~LogReaderPort ()
{
	Close ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Port management
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogReaderPort::Open ()
{
	if (_open)
		throw PortException ("Attempt to open already-opened port.");

	// Store the open time for timing purposes
	_logFile->ResetFile ();
	_open = true;

	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Port is open" << endl;
}

void LogReaderPort::Close ()
{
	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Closing port" << endl;

	_open = false;

	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Port closed" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t LogReaderPort::Read (void * const buffer, size_t count)
{
	CheckPort (true);

	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Going to read " << count << " bytes" << endl;

	// Ask the log file to give us as much data as it has available up to now (but <=count). If
	// there isn't data available immediately then have it sleep until some is, within the timeout.
	ssize_t receivedBytes;
	receivedBytes = _logFile->Read (buffer, count, _timeout);

	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Read " << receivedBytes << " bytes" << endl;

	return receivedBytes;
}

ssize_t LogReaderPort::ReadFull (void * const buffer, size_t count)
{
	size_t receivedBytes = 0;
	Timeout oldTimeout = _timeout;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "LogReaderPort::" << __func__ << "() Going to read until have " << count <<
			" bytes" << endl;
	}

	// Set the timeout to infinite blocking
	SetTimeout (Timeout (-1, 0));
	// Keep calling _logFile->Read() until count bytes have been received
	while (receivedBytes < count)
	{
		ssize_t numReceived = _logFile->Read (&(reinterpret_cast<uint8_t*> (buffer)[receivedBytes]),
											count - receivedBytes, _timeout);
		if (numReceived < 0)
		{
			// Timed out (how?!)
			// Restore the timeout
			SetTimeout (oldTimeout);
			// Throw up
			throw PortException (string ("LogReaderPort::") + __func__ +
					string (" Read() timed out, probably shouldn't happen."));
		}
		else if (numReceived == 0)
		{
			// Possibly EOF in the log file
			if (!_logFile->IsOpen ())
			{
				// Restore the timeout
				SetTimeout (oldTimeout);

				stringstream ss;
				ss << "LogReaderPort::" << __func__ << "() EOF while trying to read " <<
					count << " bytes";
				throw PortException (ss.str ());
			}
		}
		// Otherwise got data or not, continue as normal
		receivedBytes += numReceived;
	}

	// Restore the timeout
	SetTimeout (oldTimeout);
	return receivedBytes;
}

ssize_t LogReaderPort::BytesAvailable ()
{
	// Use a zero timeout.
	Timeout timeout (0, 0);
	ssize_t bytesAvailable = _logFile->BytesAvailable (timeout);
	if (_debug >= 2)
	{
		cerr << "LogReaderPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available" << endl;
	}
	return bytesAvailable;
}

ssize_t LogReaderPort::BytesAvailableWait ()
{
	// The time limit is now + the timeout
	ssize_t bytesAvailable = _logFile->BytesAvailable (_timeout);
	if (_debug >= 2)
	{
		cerr << "LogReaderPort::" << __func__ << "() Found " << bytesAvailable <<
			" bytes available" << endl;
	}
	return bytesAvailable;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t LogReaderPort::Write (const void * const buffer, size_t count)
{
	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Writing " << count << " bytes" << endl;

	// How we handle a write depends on _strictness
	size_t numWritten;
	if (_strictness == 0)
	{
		// No checking
		numWritten = count;
	}
	else if (_strictness == 1)
	{
		// Check against file, don't care about time
		if (!_logFile->CheckWrite (buffer, count, &numWritten))
		{
			throw PortException (string ("LogReaderPort::") + __func__ +
					string ("() Write check strictness 1 failed."));
		}
	}
	else
	{
		// Check against file, data from file must be within now + timeout + jitter
		Timeout timeout (_timeout);
		timeout._usec += _jitter;
		if (timeout._usec >= 1000000)
		{
			timeout._sec += timeout._usec / 1000000;
			timeout._usec %= 1000000;
		}
		if (!_logFile->CheckWrite (buffer, count, &numWritten, &timeout))
		{
			throw PortException (string ("LogReaderPort::") + __func__ +
					string ("() Write check strictness 2 failed."));
		}
	}

	if (_debug >= 2)
		cerr << "LogReaderPort::" << __func__ << "() Wrote " << numWritten << " bytes" << endl;

	return numWritten;
}

void LogReaderPort::Flush ()
{
//	_logFile->Flush ();
	// Actually shouldn't do anything here because any calls to flush on LogWriterPort didn't write
	// anything to the file. The alternative is to have LogWriterPort perform reads, write to the
	// log file, but not return that data, until there's no data left to read.
}

void LogReaderPort::Drain ()
{
//	_logFile->Drain ();
	// Actually shouldn't do anything here because any calls to drain on LogWriterPort didn't write
	// anything to the file.
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

std::string LogReaderPort::GetStatus () const
{
	stringstream status;

	status << "LogReader-specific status:" << endl;
	status << "Reading from " << _logFileName << endl;
	status << ((_open && _logFile->IsOpen ()) ? "Port is open" : "Port is closed") << endl;
	status << (_ignoreTimes ? "Ignoring file time stamps." : "Using file time stamps.") << endl;
	status << "Strictness is " << _strictness << ", with a jitter of " << _jitter << "ms" << endl;

	return Port::GetStatus () + status.str ();
}

void LogReaderPort::SetTimeout (Timeout timeout)
{
	_timeout = timeout;
}

void LogReaderPort::SetCanRead (bool canRead)
{
	_canRead = canRead;
}

void LogReaderPort::SetCanWrite (bool canWrite)
{
	_canWrite = canWrite;
}

bool LogReaderPort::IsOpen () const
{
	return _open;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LogReaderPort::ProcessOption (const std::string &option, const std::string &value)
{
	char c = '\0';

	// Check if the parent class can handle this option
	if (Port::ProcessOption (option, value))
		return true;

	if (option == "file")
	{
		_logFileName = value;
		return true;
	}
	else if (option == "ignoretimes")
	{
		_ignoreTimes = true;
		return true;
	}
	else if (option == "strictness")
	{
		istringstream is (value);
		if (!(is >> _strictness) || is.get (c) || _strictness > 2)
			throw PortException ("Bad strictness: " + value);
		return true;
	}
	else if (option == "jitter")
	{
		istringstream is (value);
		if (!(is >> _jitter) || is.get (c))
			throw PortException ("Bad jitter: " + value);
		return true;
	}

	return false;
}

void LogReaderPort::CheckPort (bool read)
{
	if (!_open || !_logFile->IsOpen ())
		throw PortException ("Port is not open.");

	if (read && !_canRead)
		throw PortException ("Cannot read from write-only port.");

	if (!read && !_canWrite)
		throw PortException ("Cannot write to read-only port.");
}

} // namespace flexiport
