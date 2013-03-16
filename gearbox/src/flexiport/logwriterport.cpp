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
#include "logwriterport.h"
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

LogWriterPort::LogWriterPort (map<string, string> options)
	: Port (), _port (NULL), _logFileName ("port.log")
{
	_type = "logwriter";

	// Look for options that we're interested in locally (file and debug)
	char c = '\0';
	for (map<string, string>::const_iterator ii = options.begin (); ii != options.end (); ii++)
	{
		if (ii->first == "file")
		{
			_logFileName = ii->second;
			options.erase ("file");     // Don't pass this one on to the underlying port
		}
		else if (ii->first == "debug")
		{
			istringstream is (ii->second);
			if (!(is >> _debug) || is.get (c))
				throw PortException ("Bad debug level: " + ii->second);
		}
	}
	// The rest of the options go on to the underlying Port object

	// Create the underlying port object
	_port = CreatePort (options);

	// Initialise the log file
	_logFile = new LogFile (_debug);
	_logFile->Open (_logFileName, false);
}

LogWriterPort::~LogWriterPort ()
{
	// _logFile should never be NULL because it's allocated in the constructor and never deleted.
	_logFile->Close ();
	delete _logFile;

	if (_port)
		_port->Close ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Port management
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogWriterPort::Open ()
{
	if (_port->IsOpen ())
		throw PortException ("Attempt to open already-opened port.");

	_port->Open ();
	_logFile->ResetFile ();

	if (_debug >= 2)
		cerr << "LogWriterPort::" << __func__ << "() Port is open" << endl;
}

void LogWriterPort::Close ()
{
	if (_debug >= 2)
		cerr << "LogWriterPort::" << __func__ << "() Closing port" << endl;

	_port->Close ();

	if (_debug >= 2)
		cerr << "LogWriterPort::" << __func__ << "() Port closed" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t LogWriterPort::Read (void * const buffer, size_t count)
{
	ssize_t receivedBytes;

	// Read from the underlying port
	receivedBytes = _port->Read (buffer, count);
	if (receivedBytes > 0)
	{
		// Write a chunk representing this read
		_logFile->WriteRead (buffer, receivedBytes);
	}

	return receivedBytes;
}

ssize_t LogWriterPort::ReadFull (void * const buffer, size_t count)
{
	ssize_t receivedBytes;

	// Read from the underlying port
	receivedBytes = _port->ReadFull (buffer, count);
	if (receivedBytes > 0)
	{
		// Write a chunk representing this read
		_logFile->WriteRead (buffer, receivedBytes);
	}

	return receivedBytes;
}

ssize_t LogWriterPort::Skip (size_t count)
{
	size_t numRead = 0, numToRead = 0;
	uint8_t bytes[32];

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "LogWriterPort::" << __func__ << "() Skipping " << count << " bytes." << endl;
	}
	// Read up to 32 bytes at a time until either a timeout occurs or we hit the terminator byte
	while (numRead < count)
	{
		ssize_t result = 0;
		numToRead = (count - numRead) > 32 ? 32 : (count - numRead);
		if ((result = Read (bytes, numToRead)) < 0)
			return -1; // Timeout
		else if (result > 0)
		{
			if (_debug >= 2)
				cerr << "LogWriterPort::" << __func__ << "() Read " << numRead << " bytes." << endl;
			numRead += result;
		}
		else
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
			{
				cerr << "LogWriterPort::" << __func__ << "() Got no data when in blocking mode."
					<< endl;
			}
			return 0;
		}
	}

	return numRead;
}

ssize_t LogWriterPort::SkipUntil (uint8_t terminator, unsigned int count)
{
	size_t numRead = 0;
	unsigned int terminatorCount = 0;
	uint8_t byte;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "LogWriterPort::" << __func__ << "() Skipping until '" << terminator <<
			"' is seen " << count << " times." << endl;
	}
	// Read bytes one at a time until either a timeout occurs or we hit the terminator byte
	while (terminatorCount < count)
	{
		ssize_t result = 0;
		if ((result = Read (&byte, 1)) < 0)
			return -1; // Timeout
		else if (result > 0)
		{
			if (_debug >= 2)
				cerr << "LogWriterPort::" << __func__ << "() Read " << result << " bytes." << endl;
			numRead++;
			if (byte == terminator)
			{
				if (_debug >= 2)
					cerr << "LogWriterPort::" << __func__ << "() Got terminator character." << endl;
				terminatorCount++;
			}
		}
		else
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
			{
				cerr << "LogWriterPort::" << __func__ << "() Got no data when in blocking mode" <<
					endl;
			}
			return 0;
		}
	}

	if (_debug >= 2)
		cerr << "LogWriterPort::" << __func__ << "() All terminators found." << endl;
	return numRead;
}

ssize_t LogWriterPort::BytesAvailable ()
{
	return _port->BytesAvailable ();
}

ssize_t LogWriterPort::BytesAvailableWait ()
{
	return _port->BytesAvailableWait ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t LogWriterPort::Write (const void * const buffer, size_t count)
{
	ssize_t numSent = 0;

	// Write to underlying port
	numSent = _port->Write (buffer, count);
	if (numSent > 0)
	{
		// Write a chunk representing this write
		_logFile->WriteWrite (buffer, numSent);
	}

	return numSent;
}

void LogWriterPort::Flush ()
{
	_port->Flush ();
}

void LogWriterPort::Drain ()
{
	_port->Drain ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

std::string LogWriterPort::GetStatus () const
{
	stringstream status;

	status << "LogWriter-specific status:" << endl;
	status << "Writing to " << _logFileName << endl;

	return _port->GetStatus () + status.str ();
}

void LogWriterPort::SetTimeout (Timeout timeout)
{
	_port->SetTimeout (timeout);
}

void LogWriterPort::SetCanRead (bool canRead)
{
	_port->SetCanRead (canRead);
//	_logFile->WritePermissionsChunk (canRead, _port->GetCanWrite ());
}

void LogWriterPort::SetCanWrite (bool canWrite)
{
	_port->SetCanWrite (canWrite);
//	_logFile->WritePermissionsChunk (_port->GetCanRead (), canWrite);
}

bool LogWriterPort::IsOpen () const
{
	if (_port == NULL)
		return false;
	return _port->IsOpen ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogWriterPort::CheckPort (bool read)
{
	if (!_port->IsOpen ())
		throw PortException ("Port is not open.");

	if (read && !_port->CanRead ())
		throw PortException ("Cannot read from write-only port.");

	if (!read && !_port->CanWrite ())
		throw PortException ("Cannot write to read-only port.");
}

} // namespace flexiport
