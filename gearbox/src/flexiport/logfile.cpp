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
#include "logfile.h"

#if defined (WIN32)
	#include <windows.h> // For Sleep()
#else
	#include <arpa/inet.h>
	#include <unistd.h>
#endif
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iostream>
using namespace std;

#if defined (WIN32)
	#define __func__        __FUNCTION__
	#define timeradd(a, b, result)                           \
		do {                                                 \
			(result)->tv_sec = (a)->tv_sec + (b)->tv_sec;    \
			(result)->tv_usec = (a)->tv_usec + (b)->tv_usec; \
			if ((result)->tv_usec >= 1000000)                \
			{                                                \
				++(result)->tv_sec;                          \
				(result)->tv_usec -= 1000000;                \
			}                                                \
		} while (0)
	#define timersub(a, b, result)                           \
		do {                                                 \
			(result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
			(result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
			if ((result)->tv_usec < 0) {                     \
				--(result)->tv_sec;                          \
				(result)->tv_usec += 1000000;                \
			}                                                \
		} while (0)
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

// Chunk header is two uint32_t's for the time stamp (seconds and microseconds) + one uint32_t for
// the data length.
const size_t CHUNK_HEADER_SIZE = (sizeof (uint32_t) * 2) + sizeof (uint32_t);


////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor/destructor
////////////////////////////////////////////////////////////////////////////////////////////////////

LogFile::LogFile (unsigned int debug)
	: _read (false), _readFile (NULL), _writeFile (NULL), _readFileSize (0), _writeFileSize (0),
	_debug (debug), _readUsage (0), _writeUsage (0), _readSize (0), _writeSize (0),
	_readBuffer (NULL), _writeBuffer (NULL), _ignoreTimes (false)
{
	timerclear (&_openTime);
}

LogFile::~LogFile ()
{
	Close ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// File management
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogFile::Open (string fileName, bool read, bool ignoreTimes)
{
	Close ();

	_fileName = fileName;
	_read = read;
	_ignoreTimes = ignoreTimes;

	if (_debug >= 2)
	{
		cerr << "LogFile::" << __func__ << "() Opening " << _fileName << " for " <<
			(_read ? "reading." : "writing.") << endl;
	}

	if (_read)
	{
		if ((_readFile = fopen ((fileName + "r").c_str (), "rb")) == NULL)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fopen(_readFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		if ((_writeFile = fopen ((fileName + "w").c_str (), "rb")) == NULL)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fopen(_writeFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		// Get the file sizes for open checks
		_readFileSize = GetFileSize (_readFile);
		_writeFileSize = GetFileSize (_writeFile);
	}
	else
	{
		if ((_readFile = fopen ((fileName + "r").c_str (), "wb")) == NULL)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fopen(_readFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		if ((_writeFile = fopen ((fileName + "w").c_str (), "wb")) == NULL)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fopen(_writeFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}

	if (_debug >= 1)
	{
		cerr << "LogFile::" << __func__ << "() Opened " << _fileName << "r/w for " <<
			(_read ? "reading." : "writing.") << endl;
	}
}

void LogFile::Close ()
{
	if (_readFile != NULL)
	{
		if (fclose (_readFile) == EOF)
		{
			_readFile = NULL;
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fclose(_readFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		_readFile = NULL;
	}
	if (_writeFile != NULL)
	{
		if (fclose (_writeFile) == EOF)
		{
			_writeFile = NULL;
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fclose(_writeFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		_writeFile = NULL;
	}

	if (_readBuffer != NULL)
	{
		free (_readBuffer);
		_readBuffer = NULL;
	}
	if (_writeBuffer != NULL)
	{
		free (_writeBuffer);
		_writeBuffer = NULL;
	}

	if (_debug >= 1)
		cerr << "LogFile::" << __func__ << "() Closed file." << endl;
}

bool LogFile::IsOpen () const
{
	if (_readFile == NULL || _writeFile == NULL)
		return false;

	long pos;
	if ((pos = ftell (_readFile)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (pos == _readFileSize)
		return false;

	if ((pos = ftell (_writeFile)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (pos == _writeFileSize)
		return false;

	return true;
}

void LogFile::ResetFile ()
{
	// Rewind file positions
	if (fseek (_readFile, 0, SEEK_SET) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek(_readFile) error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (fseek (_writeFile, 0, SEEK_SET) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek(_writeFile) error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	// Free buffers
	DeallocateReadBuffer ();
	DeallocateWriteBuffer ();

	// Reset file open time
#if defined (WIN32)
	SYSTEMTIME sysTime;
	GetSystemTime (&sysTime);
	_openTime.tv_sec = sysTime.wSecond;
	_openTime.tv_usec = sysTime.wMilliseconds * 1000;
#else
	if (gettimeofday (&_openTime, NULL) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() gettimeofday() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	if (_debug >= 1)
		cerr << "LogFile::" << __func__ << "() Reset file." << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Chunk reading (this stuff is messy)
////////////////////////////////////////////////////////////////////////////////////////////////////

ssize_t LogFile::Read (void *data, size_t count, Timeout &timeout)
{
	size_t totalRead = 0;

	// Get the current file time
	struct timeval now;
	GetCurrentFileTime (now);

	if (_debug >= 2)
		cerr << "LogFile::" << __func__ << "() Reading up to " << count << " bytes." << endl;

	// First copy any data in the overflow buffer
	if (_readUsage > 0)
	{
		if (_debug >= 2)
		{
			cerr << "LogFile::" << __func__ << "() Getting " << _readUsage <<
				" bytes from overflow buffer." << endl;
		}

		size_t toCopy = (count < _readUsage) ? count : _readUsage;
		memcpy (data, _readBuffer, toCopy);
		totalRead += toCopy;
		// Check if we've used up the stuff in the buffer
		if (count < _readUsage)
		{
			// Shift the remaining data in the overflow buffer to the beginning
			memmove (_readBuffer, &(reinterpret_cast<uint8_t*> (_readBuffer)[count]),
					_readUsage - count);
			_readUsage -= count;
			// Have all the data we need, return
			return count;
		}
		else if (count == _readUsage)
		{
			DeallocateReadBuffer ();
			// Have all the data we need, return
			return count;
		}
		else // count > _readUsage
		{
			DeallocateReadBuffer ();
			// We haven't met count yet
			count -= _readUsage;
			data = reinterpret_cast<uint8_t*> (data) + _readUsage;
		}
	}

	// Now get data from the file
	if (_ignoreTimes)
	{
		// Read chunks until we have enough data
		while (count > 0 && IsOpen ())
		{
			struct timeval timestamp;
			size_t size, read;
			read = GetSingleChunk (_readFile, &(reinterpret_cast<uint8_t*> (data)[count]), count,
									timestamp, size);
			count -= read;
			data = reinterpret_cast<uint8_t*> (data) + read;
			totalRead += read;
		}
		return totalRead;
	}
	else
	{
		// Have to pay attention to time stamps (annoying and messy)
		if (DataAvailableWithinLimit (_readFile, now))
		{
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Data available in file now." << endl;

			// Get all the data that is immediatly available and return it
			totalRead += GetChunksToTimeLimit (_readFile, data, count, now);
			return totalRead;
		}

		// There was no data instantly, so now the timeout gets involved
		if (timeout._sec == -1)
		{
			// Infinite timeout
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Getting next chunk, no timeout." << endl;

			// Get the next available chunk from the file
			struct timeval timestamp;
			size_t size;
			totalRead += GetSingleChunk (_readFile, data, count, timestamp, size);

			// Calculate the time difference between now and this chunk's timestamp
			struct timeval diff;
			GetCurrentFileTime (now);
			timersub (&timestamp, &now, &diff);
			if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
			{
				// Sleep for this period of time
				struct timespec diff2;
				diff2.tv_sec = diff.tv_sec;
				diff2.tv_nsec = diff.tv_usec * 1000;
				if (_debug >= 2)
				{
					cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec << "s " <<
						diff2.tv_nsec << "ns." << endl;
				}
#if defined (WIN32)
				DWORD sleepTime = 0;
				if (diff2.tv_sec > 0)
					sleepTime += diff2.tv_sec * 1000;
				if (diff2.tv_nsec > 0)
					sleepTime += diff2.tv_nsec / 1000000;
				Sleep (sleepTime);
#else
				nanosleep (&diff2, NULL);
#endif
			}
			return totalRead;
		}
		else if (timeout._sec > 0 || timeout._usec > 0)
		{
			// Limited timeout
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Getting next chunk with timeout." << endl;

			// Check if there is actually data available within this time limit
			struct timeval timeoutVal, limit;
			timeout.AsTimeval (timeoutVal);
			GetCurrentFileTime (now);
			timeradd (&now, &timeoutVal, &limit);
			if (DataAvailableWithinLimit (_readFile, limit))
			{
				// There is so get it
				struct timeval timestamp;
				size_t size;
				totalRead += GetSingleChunk (_readFile, data, count, timestamp, size);

				// Calculate the time difference between now and this chunk's timestamp
				struct timeval diff;
				GetCurrentFileTime (now);
				timersub (&timestamp, &now, &diff);
				if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
				{
					// Sleep for this period of time
					struct timespec diff2;
					diff2.tv_sec = diff.tv_sec;
					diff2.tv_nsec = diff.tv_usec * 1000;
					if (_debug >= 2)
					{
						cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec << "s " <<
							diff2.tv_nsec << "ns." << endl;
					}
#if defined (WIN32)
					DWORD sleepTime = 0;
					if (diff2.tv_sec > 0)
						sleepTime += diff2.tv_sec * 1000;
					if (diff2.tv_nsec > 0)
						sleepTime += diff2.tv_nsec / 1000000;
					Sleep (sleepTime);
#else
					nanosleep (&diff2, NULL);
#endif
				}
				return totalRead;
			}
			else
			{
				if (_debug >= 2)
					cerr << "LogFile::" << __func__ << "() No chunks within timeout." << endl;
				// No data available, return timeout
				return -1;
			}
		}
		else
		{
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() No data available at all." << endl;
			// No data available, return timeout
			return -1;
		}
	}
}

ssize_t LogFile::BytesAvailable (const Timeout &timeout)
{
	if (_ignoreTimes)
	{
		// Don't care about times, so just get the file size.
		return GetFileSize (_readFile);
	}
	else
	{
		// Get the current file time
		struct timeval now;
		GetCurrentFileTime (now);

		// Now count data from the file
		if (DataAvailableWithinLimit (_readFile, now))
		{
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Enough data available in file now." << endl;
			// Data available immediately, return its size plus the size of the overflow buffer
			return _readUsage + GetChunkSizesToTimeLimit (_readFile, now);
		}
		else if (_readUsage > 0)
		{
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Only data in overflow buffer." << endl;
			// No data from file, but there is data in the overflow buffer, so that will do
			return _readUsage;
		}

		// There was no data instantly, so now the timeout gets involved
		if (timeout._sec == -1)
		{
			// Infinite timeout
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Getting next chunk, no timeout." << endl;

			// Get the next available chunk from the file
			struct timeval timeStamp;
			size_t size;
			GetNextChunkInfo (_readFile, timeStamp, size);

			// Calculate the time difference between now and this chunk's timestamp
			struct timeval diff;
			GetCurrentFileTime (now);
			timersub (&timeStamp, &now, &diff);
			if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
			{
				// Sleep for this period of time
				struct timespec diff2;
				diff2.tv_sec = diff.tv_sec;
				diff2.tv_nsec = diff.tv_usec * 1000;
				if (_debug >= 2)
				{
					cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec << "s " <<
						diff2.tv_nsec << "ns." << endl;
				}
#if defined (WIN32)
				DWORD sleepTime = 0;
				if (diff2.tv_sec > 0)
					sleepTime += diff2.tv_sec * 1000;
				if (diff2.tv_nsec > 0)
					sleepTime += diff2.tv_nsec / 1000000;
				Sleep (sleepTime);
#else
				nanosleep (&diff2, NULL);
#endif
			}
			return size;
		}
		else if (timeout._sec > 0 || timeout._usec > 0)
		{
			// Limited timeout
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() Getting next chunk with timeout." << endl;

			// Check if there is actually data available within this time limit
			struct timeval timeoutVal, limit;
			timeout.AsTimeval (timeoutVal);
			GetCurrentFileTime (now);
			timeradd (&now, &timeoutVal, &limit);
			if (DataAvailableWithinLimit (_readFile, limit))
			{
				// There is so get it
				struct timeval timeStamp;
				size_t size;
				GetNextChunkInfo (_readFile, timeStamp, size);

				// Calculate the time difference between now and this chunk's timestamp
				struct timeval diff;
				GetCurrentFileTime (now);
				timersub (&timeStamp, &now, &diff);
				if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
				{
					// Sleep for this period of time
					struct timespec diff2;
					diff2.tv_sec = diff.tv_sec;
					diff2.tv_nsec = diff.tv_usec * 1000;
					if (_debug >= 2)
					{
						cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec << "s " <<
							diff2.tv_nsec << "ns." << endl;
					}
#if defined (WIN32)
					DWORD sleepTime = 0;
					if (diff2.tv_sec > 0)
						sleepTime += diff2.tv_sec * 1000;
					if (diff2.tv_nsec > 0)
						sleepTime += diff2.tv_nsec / 1000000;
					Sleep (sleepTime);
#else
					nanosleep (&diff2, NULL);
#endif
				}
				return size;
			}
			else
			{
				if (_debug >= 2)
					cerr << "LogFile::" << __func__ << "() No chunks within timeout." << endl;
				// No data available, return timeout
				return -1;
			}
		}
		else
		{
			if (_debug >= 2)
				cerr << "LogFile::" << __func__ << "() No data available at all." << endl;
			// No data available, return timeout
			return -1;
		}
	}
}

bool LogFile::CheckWrite (const void * const data, const size_t count, size_t * const numWritten,
				const Timeout * const timeout)
{
	size_t totalRead = 0;
	long readFileOffset;
	// Get the current read file offset (see comment at function end)
	if ((readFileOffset = ftell (_readFile)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 2)
	{
		cerr << "LogFile::" << __func__ << "() Checking " << count <<
			" bytes for accuracy. Timeouts will " << ((timeout == NULL) ? "" : "not ") <<
			"be used." << endl;
	}

	// Allocate space to store the data to compare with
	uint8_t *fileData;
	fileData = new uint8_t[count];

	// Pull any data out of the overflow first
	bool needMore = true;
	if (_writeUsage > 0)
	{
		if (_debug >= 2)
		{
			cerr << "LogFile::" << __func__ << "() Getting " << _writeUsage <<
				" bytes from overflow buffer." << endl;
		}

		size_t toCopy = (count < _writeUsage) ? count : _writeUsage;
		memcpy (fileData, _writeBuffer, toCopy);
		totalRead += toCopy;
		// Check if we've used up the stuff in the buffer
		if (count < _writeUsage)
		{
			// Shift the remaining data in the buffer to the beginning
			memmove (_writeBuffer, &(reinterpret_cast<uint8_t*> (_writeBuffer)[count]),
					_writeUsage - count);
			_writeUsage -= count;
			// Have all the data we need
			needMore = false;
		}
		else if (count == _writeUsage)
		{
			DeallocateWriteBuffer ();
			// Have all the data we need
			needMore = false;
		}
		else // count > _writeUsage
		{
			DeallocateWriteBuffer ();
			// We haven't met count yet so get the rest from the file
		}
	}

	if (needMore)
	{
		if (timeout == NULL || _ignoreTimes)
		{
			// Don't care about times, so just get data and compare
			while (totalRead < count && IsOpen ())
			{
				struct timeval timestamp;
				size_t size, read;
				read = GetSingleChunk (_writeFile, &fileData[totalRead], count - totalRead,
										timestamp, size);
				totalRead += read;
			}
		}
		else
		{
			// Things are more complex in this case
			if (timeout->_sec == -1)
			{
				// Infinite timeout - ignore jitter

				// Get the next available chunk from the file
				struct timeval timestamp;
				size_t size, read;
				read = GetSingleChunk (_writeFile, &fileData[totalRead], count - totalRead,
										timestamp, size);
				totalRead += read;

				// Calculate the time difference between now and this chunk's timestamp
				struct timeval diff, now;
				GetCurrentFileTime (now);
				timersub (&timestamp, &now, &diff);
				if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
				{
					// Sleep for this period of time
					struct timespec diff2;
					diff2.tv_sec = diff.tv_sec;
					diff2.tv_nsec = diff.tv_usec * 1000;
					if (_debug >= 2)
					{
						cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec << "s " <<
							diff2.tv_nsec << "ns." << endl;
					}
#if defined (WIN32)
					DWORD sleepTime = 0;
					if (diff2.tv_sec > 0)
						sleepTime += diff2.tv_sec * 1000;
					if (diff2.tv_nsec > 0)
						sleepTime += diff2.tv_nsec / 1000000;
					Sleep (sleepTime);
#else
					nanosleep (&diff2, NULL);
#endif
				}
			}
			else if (timeout->_sec > 0 || timeout->_usec > 0)
			{
				// Limited timeout

				// Check if there is actually data available within this time limit
				struct timeval now, timeoutVal, limit;
				timeout->AsTimeval (timeoutVal);
				GetCurrentFileTime (now);
				timeradd (&now, &timeoutVal, &limit);
				if (DataAvailableWithinLimit (_writeFile, limit))
				{
					// There is so get it
					struct timeval timestamp;
					size_t size, read;
					read = GetSingleChunk (_writeFile, &fileData[totalRead], count - totalRead,
											timestamp, size);
					totalRead += read;

					// Calculate the time difference between now and this chunk's timestamp
					struct timeval diff;
					GetCurrentFileTime (now);
					timersub (&timestamp, &now, &diff);
					if (diff.tv_sec >= 0 && diff.tv_usec >= 0)
					{
						// Sleep for this period of time
						struct timespec diff2;
						diff2.tv_sec = diff.tv_sec;
						diff2.tv_nsec = diff.tv_usec * 1000;
						if (_debug >= 2)
						{
							cerr << "LogFile::" << __func__ << "() Sleeping for " << diff2.tv_sec <<
								"s " << diff2.tv_nsec << "ns." << endl;
						}
#if defined (WIN32)
						DWORD sleepTime = 0;
						if (diff2.tv_sec > 0)
							sleepTime += diff2.tv_sec * 1000;
						if (diff2.tv_nsec > 0)
							sleepTime += diff2.tv_nsec / 1000000;
						Sleep (sleepTime);
#else
						nanosleep (&diff2, NULL);
#endif
					}
				}
				// else no data available
			}
			// else no data available
		}
	}

	// At this point, the fileData buffer is full with the right quantity of data (or less if less
	// was available). Compare it to the given data and return the result;
	bool result;
	if (totalRead < count)
		result = false;
	else if (memcmp (data, fileData, count) != 0)
		result = false;
	else
		result = true;

	// One final thing to do: it's possible that the write file has reached its end and so been
	// closed. This means that any attempts to read the response to the write will now fail. To
	// prevent this, reopen the files and re-position the read file's offset to where it was. Any
	// writes after this would have choked anyway since the file is at an end so moving the write
	// file to 1 byte before the end is unlikely to cause any problems, while still registering as
	// an open file.
	if (!IsOpen ())
	{
		Open (_fileName, _read, _ignoreTimes);
		if (fseek (_readFile, readFileOffset, SEEK_SET) < 0)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fseek(_readFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		if (fseek (_writeFile, -1, SEEK_END) < 0)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fseek(_writeFile) error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
	}

	delete[] fileData;
	*numWritten = totalRead;
	return result;
}

void LogFile::Flush ()
{
	// Dump the read overflow buffer
	DeallocateWriteBuffer();

	// If there is data available in the read file, skip passed it
	struct timeval now, timeStamp;
	size_t size;
	GetCurrentFileTime (now);
	if (DataAvailableWithinLimit (_readFile, now))
	{
		GetNextChunkInfo (_readFile, timeStamp, size);
		do
		{
			if (_debug >= 2)
			{
				cerr << "LogFile::" << __func__ << "() Skipping " << CHUNK_HEADER_SIZE + size <<
					"bytes in _readFile." << endl;
			}
			if (fseek (_readFile, CHUNK_HEADER_SIZE + size, SEEK_CUR) < 0)
			{
				stringstream ss;
				ss << "LogFile::" << __func__ << "() fseek(_readFile) error: (" << ErrNo () <<
					") " << StrError (ErrNo ());
				throw PortException (ss.str ());
			}
			GetNextChunkInfo (_readFile, timeStamp, size);
		}
		while (!(timercmp (&timeStamp, &now, >)) && IsOpen ());
	}

	// Call Drain to do the same on the write file
	Drain ();
}

void LogFile::Drain ()
{
	// Dump the write overflow buffer
	DeallocateWriteBuffer();

	// If there is data available in the write file, skip passed it
	struct timeval now, timeStamp;
	size_t size;
	GetCurrentFileTime (now);
	if (DataAvailableWithinLimit (_readFile, now))
	{
		GetNextChunkInfo (_writeFile, timeStamp, size);
		do
		{
			if (_debug >= 2)
			{
				cerr << "LogFile::" << __func__ << "() Skipping " << CHUNK_HEADER_SIZE + size <<
					"bytes in _writeFile." << endl;
			}
			if (fseek (_writeFile, CHUNK_HEADER_SIZE + size, SEEK_CUR) < 0)
			{
				stringstream ss;
				ss << "LogFile::" << __func__ << "() fseek(_writeFile) error: (" << ErrNo () <<
					") " << StrError (ErrNo ());
				throw PortException (ss.str ());
			}
			GetNextChunkInfo (_writeFile, timeStamp, size);
		}
		while (!(timercmp (&timeStamp, &now, >)) && IsOpen ());
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// File writing (this stuff is easy)
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogFile::WriteRead (const void * const data, size_t count)
{
	if (_debug >= 1)
	{
		cerr << "LogFile::" << __func__ << "() Writing read chunk of size " << count <<
			" bytes." << endl;
	}
	WriteTimeStamp (_readFile);
	uint32_t temp = htonl (static_cast<uint32_t> (count));
	WriteToFile (_readFile, &temp, sizeof (temp));
	WriteToFile (_readFile, data, count);
}

void LogFile::WriteWrite (const void * const data, size_t count)
{
	if (_debug >= 1)
	{
		cerr << "LogFile::" << __func__ << "() Writing write chunk of size " << count <<
			" bytes." << endl;
	}
	WriteTimeStamp (_writeFile);
	uint32_t temp = htonl (static_cast<uint32_t> (count));
	WriteToFile (_writeFile, &temp, sizeof (temp));
	WriteToFile (_writeFile, data, count);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void LogFile::AllocateReadBuffer (unsigned int size)
{
	// Allocate based on size
	if (_readBuffer == NULL && size > 0)
	{
		// Allocate a new buffer
		if ((_readBuffer = reinterpret_cast<uint8_t*> (malloc (sizeof (uint8_t) * size))) ==
			NULL)
		{
			throw PortException (string ("LogFile::") + __func__ +
					string ("() Failed to allocate memory for read buffer."));
		}
		_readUsage = 0;
	}
	else if (_readBuffer != NULL && size > 0)
	{
		// Reallocate
		uint8_t *newBuffer;
		if ((newBuffer = reinterpret_cast<uint8_t*> (realloc (_readBuffer, sizeof (uint8_t) *
				size))) == NULL)
		{
			free (_readBuffer);
			_readBuffer = NULL;
			throw PortException (string ("LogFile::") + __func__ +
					string ("() Failed to allocate memory for read buffer."));
		}
		_readBuffer = newBuffer;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Read overflow buffer was reallocated." << endl;
	}
	else if (_readBuffer != NULL && size == 0)
	{
		// Free
		free (_readBuffer);
		_readBuffer = NULL;
		_readUsage = 0;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Read overflow buffer was freed." << endl;
	}
	_readSize = size;
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Allocated read overflow buffer of size " <<
			_readSize << "bytes." << endl;
	}
}

void LogFile::AllocateWriteBuffer (unsigned int size)
{
	// Allocate based on size
	if (_writeBuffer == NULL && size > 0)
	{
		// Allocate a new buffer
		if ((_writeBuffer = reinterpret_cast<uint8_t*> (malloc (sizeof (uint8_t) * size))) ==
			NULL)
		{
			throw PortException (string ("LogFile::") + __func__ +
					string ("() Failed to allocate memory for read buffer."));
		}
		_writeUsage = 0;
	}
	else if (_readBuffer != NULL && size > 0)
	{
		// Reallocate
		uint8_t *newBuffer;
		if ((newBuffer = reinterpret_cast<uint8_t*> (realloc (_writeBuffer, sizeof (uint8_t) *
				size))) == NULL)
		{
			free (_writeBuffer);
			_writeBuffer = NULL;
			throw PortException (string ("LogFile::") + __func__ +
					string ("() Failed to allocate memory for read buffer."));
		}
		_writeBuffer = newBuffer;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Write overflow buffer was reallocated." << endl;
	}
	else if (_writeBuffer != NULL && size == 0)
	{
		// Free
		free (_writeBuffer);
		_writeBuffer = NULL;
		_writeUsage = 0;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Write overflow buffer was freed." << endl;
	}
	_writeSize = size;
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Allocated write overflow buffer of size " <<
			_writeSize << "bytes." << endl;
	}
}

void LogFile::DeallocateReadBuffer ()
{
	if (_readBuffer != NULL)
	{
		free (_readBuffer);
		_readBuffer = NULL;
		_readSize = 0;
		_readUsage = 0;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Deallocated read overflow buffer." << endl;
	}
}

void LogFile::DeallocateWriteBuffer ()
{
	if (_writeBuffer != NULL)
	{
		free (_writeBuffer);
		_writeBuffer = NULL;
		_writeSize = 0;
		_writeUsage = 0;
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Deallocated write overflow buffer." << endl;
	}
}

void LogFile::GetCurrentFileTime (struct timeval &dest)
{
	struct timeval now;

#if defined (WIN32)
	SYSTEMTIME sysTime;
	GetSystemTime (&sysTime);
	now.tv_sec = sysTime.wSecond;
	now.tv_usec = sysTime.wMilliseconds * 1000;
#else
	if (gettimeofday (&now, NULL) < 0)
	{
		stringstream ss;
		ss << "LogReaderPort::" << __func__ << "() gettimeofday() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif

	timersub (&now, &_openTime, &dest);
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Current file time is " << dest.tv_sec << "s " <<
			dest.tv_usec << "us." << endl;
	}
}

bool LogFile::DataAvailableWithinLimit (FILE * const file, const struct timeval &limit)
{
	struct timeval chunkTime;
	size_t chunkSize;

	GetNextChunkInfo (file, chunkTime, chunkSize);
	if (!(timercmp (&chunkTime, &limit, >)))
		return true;
	return false;
}

void LogFile::GetNextChunkInfo (FILE * const file, struct timeval &timeStamp, size_t &size)
{
	// Read the chunk info
	uint32_t secs, usecs, tempSize;
	ReadFromFile (file, &secs, sizeof (secs));
	ReadFromFile (file, &usecs, sizeof (usecs));
	timeStamp.tv_sec = ntohl (secs);
	timeStamp.tv_usec = ntohl (usecs);
	ReadFromFile (file, &tempSize, sizeof (tempSize));
	size = ntohl (tempSize);

	// Rewind the file back to the beginning of the chunk header
	if (fseek (file, -1 * static_cast<int> (CHUNK_HEADER_SIZE), SEEK_CUR) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Next chunk has size " << size <<
			" bytes and time stamp " << timeStamp.tv_sec << "s " << timeStamp.tv_usec <<
			"us." << endl;
	}
}

size_t LogFile::GetChunksToTimeLimit (FILE * const file, void *data, size_t count,
									const struct timeval &limit)
{
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Getting chunks up to " << limit.tv_sec << "s " <<
			limit.tv_usec << "us." << endl;
	}

	// Keep reading chunks from the file until the limit is passed or count bytes have been read.
	struct timeval chunkTime;
	size_t chunkSize, totalRead = 0;
	GetNextChunkInfo (file, chunkTime, chunkSize);
	while (!(timercmp (&chunkTime, &limit, >)) && count > 0 && IsOpen ())
	{
		size_t numSaved = GetSingleChunk (file, data, count, chunkTime, chunkSize);
		data = reinterpret_cast<uint8_t*> (data) + numSaved;
		count -= numSaved;
		totalRead += numSaved;
		// Safety check
		if (!IsOpen ())
			break;
		// Next chunk
		GetNextChunkInfo (file, chunkTime, chunkSize);
	}

	if (_debug >= 3)
		cerr << "LogFile::" << __func__ << "() Read a total of " << totalRead << " bytes." << endl;
	return totalRead;
}

size_t LogFile::GetChunkSizesToTimeLimit (FILE * const file, const struct timeval &limit)
{
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Getting chunk sizes up to " << limit.tv_sec <<
			"s " << limit.tv_usec << "us." << endl;
	}

	struct timeval chunkTime;
	size_t chunkSize, totalSize = 0;

	// Store the current file position
	long currentPos;
	if ((currentPos = ftell (file)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	GetNextChunkInfo (file, chunkTime, chunkSize);
	while (!(timercmp (&chunkTime, &limit, >)) && IsOpen ())
	{
		totalSize += chunkSize;
		// Safety check
		if (!IsOpen ())
			break;
		// Next chunk
		GetNextChunkInfo (file, chunkTime, chunkSize);
	}
	// Reopen if necessary
	if (!IsOpen ())
		Open (_fileName, _read, _ignoreTimes);

	// Go back to the start position
	if (fseek (file, currentPos, SEEK_SET) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	if (_debug >= 3)
		cerr << "LogFile::" << __func__ << "() Found a total of " << totalSize << " bytes." << endl;
	return totalSize;
}

// This function returns the number of bytes put into data. This may be less than the number of
// bytes actually read from the chunk, as some might go into an overflow buffer.
size_t LogFile::GetSingleChunk (FILE * const file, void *data, size_t count,
								struct timeval &timeStamp, size_t &size)
{
	if (_debug >= 3)
	{
		long pos;
		if ((pos = ftell (file)) < 0)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		cerr << "LogFile::" << __func__ << "() Reading a single chunk from " <<
			((file == _readFile) ? "read file" : "write file") << " at position " << pos << endl;
	}

	// Read the chunk info
	uint32_t secs, usecs, tempSize;
	ReadFromFile (file, &secs, sizeof (secs));
	ReadFromFile (file, &usecs, sizeof (usecs));
	timeStamp.tv_sec = ntohl (secs);
	timeStamp.tv_usec = ntohl (usecs);
	ReadFromFile (file, &tempSize, sizeof (tempSize));
	size = ntohl (tempSize);
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Chunk has time " << timeStamp.tv_sec << "s " <<
			timeStamp.tv_usec << "us and is " << size << " bytes." << endl;
	}

	// Check if this chunk will fit in data
	if (size > count)
	{
		if (_debug >= 3)
			cerr << "LogFile::" << __func__ << "() Chunk is too big for destination." << endl;

		// Need to allocate an overflow buffer
		uint8_t *overFlow;
		if (file == _readFile)
		{
			AllocateReadBuffer (size - count);
			overFlow = _readBuffer;
			_readUsage = size - count;
		}
		else
		{
			AllocateWriteBuffer (size - count);
			overFlow = _writeBuffer;
			_writeUsage = size - count;
		}

		// Read as much as can fit into data
		ReadFromFile (file, data, sizeof (uint8_t) * count);
		if (_debug >= 3)
		{
			cerr << "LogFile::" << __func__ << "() Read " <<
				sizeof (*reinterpret_cast<uint8_t*> (data)) * count << " bytes into destination." <<
				endl;
		}
		// Read the rest into the overflow buffer
		ReadFromFile (file, overFlow, sizeof (uint8_t) * (size - count));
		if (_debug >= 3)
		{
			cerr << "LogFile::" << __func__ << "() Read " << sizeof (uint8_t) * (size - count) <<
				" bytes into overflow buffer." << endl;
		}

		return count;
	}
	else
	{
		// It'll fit so read straight into data
		ReadFromFile (file, data, sizeof (uint8_t) * size);
		if (_debug >= 3)
		{
			cerr << "LogFile::" << __func__ << "() Read " <<
				sizeof (*reinterpret_cast<uint8_t*> (data)) * size << " bytes into destination." <<
				endl;
		}
		return size;
	}
}

size_t LogFile::GetFileSize (FILE * const file)
{
	// Store the current position
	long currentPos;
	if ((currentPos = ftell (file)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	// Seek to the end of the file and find its position
	if (fseek (file, 0, SEEK_END) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
	long fileSize;
	if ((fileSize = ftell (file)) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() ftell() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	// Go back to the original position
	if (fseek (file, currentPos, SEEK_SET) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() fseek() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}

	return fileSize;
}

void LogFile::ReadFromFile (FILE * const file, void * const dest, size_t count)
{
	if (!_read)
	{
		throw PortException (string ("LogFile::") + __func__ +
				string ("() Cannot read from write log file."));
	}

	// Test if this file is actually open
	if (file == NULL)
		throw PortException (string ("LogFile::") + __func__ + string ("() Log file is not open."));
	else if (ftell (file) < 0)
		throw PortException (string ("LogFile::") + __func__ + string ("() Log file is not open."));

	if (dest == NULL)
		throw PortException (string ("LogFile::") + __func__ + string ("() No destination."));

	size_t numRead;
	if ((numRead = fread (dest, 1, count, file)) < count)
	{
		cerr << "LogFile::" << __func__ << "() Only read " << numRead << " of " << count <<
			" bytes" << endl;
		if (feof (file))
		{
			// Close the files
			Close ();
		}
		else if (ferror (file))
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fread() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		else
		{
			throw PortException (string ("LogFile::") + __func__ +
					string ("() Didn't get enough data from fread()."));
		}
	}
}

void LogFile::WriteToFile (FILE * const file, const void * const data, size_t count)
{
	size_t totalWritten = 0;

	if (_read)
	{
		throw PortException (string ("LogFile::") + __func__ +
				string ("() Cannot write to read log file."));
	}
	if (file == NULL)
		throw PortException (string ("LogFile::") + __func__ + string ("() Log file is not open."));
	if (data == NULL)
		throw PortException (string ("LogFile::") + __func__ + string ("() No data to write."));

	while (totalWritten < count)
	{
		ssize_t numWritten;
		if ((numWritten = fwrite (&(reinterpret_cast<const uint8_t*> (data)[totalWritten]),
								sizeof (uint8_t), count - totalWritten, file)) < 0)
		{
			stringstream ss;
			ss << "LogFile::" << __func__ << "() fwrite() error: (" << ErrNo () << ") " <<
				StrError (ErrNo ());
			throw PortException (ss.str ());
		}
		totalWritten += numWritten;
	}
	if (_debug >= 3)
		cerr << "LogFile::" << __func__ << "() Wrote " << totalWritten << " bytes." << endl;
}

void LogFile::WriteTimeStamp (FILE * const file)
{
	// Calculate the time difference between now and the time the file was opened
	struct timeval now, diff;
#if defined (WIN32)
	SYSTEMTIME sysTime;
	GetSystemTime (&sysTime);
	now.tv_sec = sysTime.wSecond;
	now.tv_usec = sysTime.wMilliseconds * 1000;
#else
	if (gettimeofday (&now, NULL) < 0)
	{
		stringstream ss;
		ss << "LogFile::" << __func__ << "() gettimeofday() error: (" << ErrNo () << ") " <<
			StrError (ErrNo ());
		throw PortException (ss.str ());
	}
#endif
	timersub (&now, &_openTime, &diff);
	uint32_t secs, usecs;
	secs = htonl (static_cast<uint32_t> (diff.tv_sec));
	usecs = htonl (static_cast<uint32_t> (diff.tv_usec));
	// Write the time stamp to the file
	WriteToFile (file, &secs, sizeof (secs));
	WriteToFile (file, &usecs, sizeof (usecs));
	if (_debug >= 3)
	{
		cerr << "LogFile::" << __func__ << "() Wrote time stamp: " << diff.tv_sec << "s " <<
			diff.tv_usec << "us (time of write is " << now.tv_sec << "s " << now.tv_usec << "us)."
			<< endl;
	}
}

} // namespace flexiport
