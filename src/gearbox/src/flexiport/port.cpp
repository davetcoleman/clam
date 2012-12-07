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

#include "port.h"
#include "flexiport.h"

#include <cstring>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <sstream>
#include <iostream>
#include <iomanip>
using namespace std;

#if defined (WIN32)
	#define __func__    __FUNCTION__
#endif

namespace flexiport
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////////////////

Port::Port ()
	: _type ("none"), _debug (0), _timeout (-1, 0), _canRead (true),
	_canWrite (true), _alwaysOpen (false)
{
}

Port::Port (unsigned int debug, Timeout timeout,
			bool canRead, bool canWrite, bool alwaysOpen)
	: _type ("none"), _debug (debug), _timeout (timeout), _canRead (canRead),
	_canWrite (canWrite), _alwaysOpen (alwaysOpen)
{
}

Port::~Port ()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Read functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t Port::ReadString (std::string &buffer)
{
	char *charBuffer = NULL;
	ssize_t bytesAvailable = 0, numRead = 0;

	buffer.clear ();
	CheckPort (true);

	// Wait for some data to be available
	bytesAvailable = BytesAvailableWait ();
	if (bytesAvailable < 0)
		return -1; // Timeout
	else if (bytesAvailable == 0)
		return -1; // Nothing available

	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Got " << bytesAvailable <<
			" bytes waiting to be read into a string" << endl;
	}

	// Read this many characters into a string - include space for a NULL in case one doesn't
	// come in the transmitted data.
	charBuffer = new char[bytesAvailable + 1];
	if ((numRead = Read (charBuffer, bytesAvailable)) < 0)
		return -1; // Timeout
	charBuffer[numRead] = '\0';
	buffer = charBuffer;

	if (numRead != bytesAvailable && _debug >= 1)
	{
		cerr << "WARNING: Port::" << __func__ <<
			" Read different number of bytes than peek said were available: " <<
			numRead << " != " << bytesAvailable << endl;
	}
	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Read " << buffer.size () + 1 <<
			" bytes into a string." << endl;
	}
	return buffer.size ();
}

ssize_t Port::ReadUntil (void * const buffer, size_t count, uint8_t terminator)
{
	size_t numRead = 0;
	uint8_t byte;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Reading until '" << terminator << "' or " <<
			count << " bytes." << endl;
	}
	// Read bytes one at a time until either a timeout occurs, we hit the terminator byte, or
	// we exhaust the buffer
	while (numRead < count)
	{
		ssize_t result = 0;
		if ((result = Read (&byte, 1)) < 0)
			return -1; // Timeout
		else if (result > 0)
		{
			if (_debug >= 2)
				cerr << "Port::" << __func__ << "() Read " << result << " bytes." << endl;
			reinterpret_cast<uint8_t*> (buffer)[numRead] = byte;
			numRead++;
			if (byte == terminator)
			{
				if (_debug >= 2)
					cerr << "Port::" << __func__ << "() Got terminator character." << endl;
				// Got the terminator so stop reading now
				break;
			}
		}
		else
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
				cerr << "Port::" << __func__ << "() Got no data when in blocking mode." << endl;
			return 0;
		}
	}

	return numRead;
}

ssize_t Port::ReadStringUntil (std::string &buffer, char terminator)
{
	char c;

	buffer.clear ();
	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Reading string until receive '" << terminator <<
			"'" << endl;
	}
	// Read bytes one at a time until either a timeout occurs or we hit the terminator byte
	while (true)
	{
		ssize_t result = 0;
		if ((result = Read (&c, 1)) < 0)
			return -1; // Timeout
		else if (result > 0)
		{
			buffer += c;
			if (c == terminator)
			{
				if (_debug >= 2)
					cerr << "Port::" << __func__ << "() Got terminator char" << endl;
				// Got the terminator so stop reading now
				break;
			}
		}
		else
		{
			// No data received and didn't timeout
			if (IsBlocking ())
				cerr << "Port::" << __func__ << "() Got no data when in blocking mode" << endl;
			return 0;
		}
	}

	return buffer.size ();
}

ssize_t Port::ReadLine (char * const buffer, size_t count)
{
	ssize_t numRead = ReadUntil (buffer, count - 1, '\n');
	if (numRead >= 0)
		buffer[numRead] = '\0';
	return numRead;
}

ssize_t Port::Skip (size_t count)
{
	size_t numRead = 0, numToRead = 0;
	uint8_t bytes[32];

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Skipping " << count << " bytes." << endl;
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
				cerr << "Port::" << __func__ << "() Read " << numRead << " bytes." << endl;
			numRead += result;
		}
		else
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
				cerr << "Port::" << __func__ << "() Got no data when in blocking mode." << endl;
			return 0;
		}
	}

	return numRead;
}

ssize_t Port::SkipUntil (uint8_t terminator, unsigned int count)
{
	size_t numRead = 0;
	unsigned int terminatorCount = 0;
	uint8_t byte;

	CheckPort (true);

	if (_debug >= 2)
	{
		cerr << "Port::" << __func__ << "() Skipping until '" << terminator << "' is seen " <<
			count << " times." << endl;
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
				cerr << "Port::" << __func__ << "() Read " << result << " bytes." << endl;
			numRead++;
			if (byte == terminator)
			{
				if (_debug >= 2)
					cerr << "Port::" << __func__ << "() Got terminator character." << endl;
				terminatorCount++;
			}
		}
		else
		{
			// No data received and didn't timeout, so must be in non-blocking mode
			if (IsBlocking ())
				cerr << "Port::" << __func__ << "() Got no data when in blocking mode" << endl;
			return 0;
		}
	}

	if (_debug >= 2)
		cerr << "Port::" << __func__ << "() All terminators found." << endl;
	return numRead;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Write functions
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t Port::WriteFull (const void * const buffer, size_t count)
{
	size_t totalWritten = 0;

	while (totalWritten != count)
	{
		ssize_t numWritten  = Write (&(reinterpret_cast<const uint8_t*> (buffer)[totalWritten]),
									count - totalWritten);
		if (numWritten == 0)
		{
			// There's a chance that the port is no longer open
			if (!IsOpen ())
			{
				stringstream ss;
				ss << "Port::" << __func__ << "() Port closed while trying to write " <<
					count << " bytes";
				throw PortException (ss.str ());
			}
			// If it is open we can keep going, but with a warning
			if (_debug >= 1)
			{
				cerr << "WARNING: Port::" << __func__ <<
					" Port closed during WriteFull operation; data may be missing/corrupted." <<
					endl;
			}
		}
		else if (numWritten > 0)
		{
			// Otherwise got data or not, continue as normal
			totalWritten += numWritten;
		}
		// Ignore timeouts, just go around again
	}

	return totalWritten;
}

ssize_t Port::WriteString (const char * const buffer)
{
	ssize_t numWritten = 0, numToWrite = strlen (buffer);

	CheckPort (false);

	if ((numWritten = Write (buffer, numToWrite)) < 0)
		return -1; // Timeout
	if (numWritten < numToWrite && _debug >= 1)
	{
		cerr << "WARNING: Port::" << __func__ << "() Did not write whole string; only wrote " <<
			numWritten << " of " << numToWrite << " bytes" << endl;
	}

	return numWritten;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Other public API functions
////////////////////////////////////////////////////////////////////////////////////////////////////

string Port::GetStatus () const
{
	stringstream status;

	status.fill ('0');
	status << "Base status:" << endl;
	status << "Debug level: " << _debug << "\tTimeout: " << _timeout._sec << "." << setw (6) <<
		_timeout._usec << endl;
	status << "Will block: " << IsBlocking ();
	status << "\tPermissions: " << ((_canRead && _canWrite) ? "rw" :
			(_canRead ? "r" : "w")) << endl;

	return status.str ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void Port::ProcessOptions (const std::map<std::string, std::string> &options)
{
	for (map<string, string>::const_iterator ii = options.begin (); ii != options.end (); ii++)
	{
		if (!ProcessOption (ii->first, ii->second))
			throw PortException ("Unknown option: " + ii->first + "=" + ii->second);
	}
	assert (_canRead || _canWrite);     // At least one of these must be true
}

bool Port::ProcessOption (const string &option, const string &value)
{
	char c = '\0';

	if (option == "debug")
	{
		istringstream is (value);
		if (!(is >> _debug) || is.get (c))	// The is.get checks if there was junk after the number
			throw PortException ("Bad debug level: " + value);
		return true;
	}

	else if (option == "timeout")
	{
		istringstream is (value);
		if (!(is >> _timeout._sec))
			throw PortException ("Bad time out: " + value);
		// Check for a decimal point
		if (is.get (c))
		{
			if (c == '.')
			{
				// Get the number after the decimal point into usecs
				if (!(is >> _timeout._usec) || is.get (c))
					throw PortException ("Bad time out: " + value);
				// Figure out how long the part after the decimal point was and use that to figure
				// out how large the value should actually be
				switch (value.size () - value.find ('.') - 1)
				{
					case 1:
						_timeout._usec *= 100000;
						break;
					case 2:
						_timeout._usec *= 10000;
						break;
					case 3:
						_timeout._usec *= 1000;
						break;
					case 4:
						_timeout._usec *= 100;
						break;
					case 5:
						_timeout._usec *= 10;
						break;
					default:
						break;
				}
				// else all OK
			}
			else
				throw PortException ("Bad time out: " + value);
		}
		return true;
	}

	else if (option == "readonly")
	{
		_canRead = true;
		_canWrite = false;
		return true;
	}
	else if (option == "writeonly")
	{
		_canRead = false;
		_canWrite = true;
		return true;
	}
	else if (option == "readwrite")
	{
		_canRead = true;
		_canWrite = true;
		return true;
	}

	else if (option == "alwaysopen")
	{
		_alwaysOpen = true;
		return true;
	}

	return false;
}

} // namespace flexiport
