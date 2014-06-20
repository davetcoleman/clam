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

#ifndef __LOGFILE_H
#define __LOGFILE_H

#if defined (WIN32)
	#include <winsock2.h> // For timeval
#else
	#include <sys/time.h>
#endif
#include <string>
#include <vector>

#include "timeout.h"
#include "flexiport_types.h"
#include <sys/types.h>

namespace flexiport
{

// Class for managing a log file pair
class LogFile
{
	public:
		LogFile (unsigned int debug);
		~LogFile ();

		void Open (std::string fileName, bool read, bool ignoreTimes = false);
		void Close ();
		bool IsOpen () const;
		void ResetFile ();

		// File reading
		ssize_t Read (void *data, size_t count, Timeout &timeout);
		ssize_t BytesAvailable (const Timeout &timeout);
		bool CheckWrite (const void * const data, const size_t count, size_t * const numWritten,
				const Timeout * const timeout = NULL);
		void Flush ();
		void Drain ();

		// File writing
		void WriteRead (const void * const data, size_t count);
		void WriteWrite (const void * const data, size_t count);

	private:
		std::string _fileName;
		bool _read;
		FILE *_readFile, *_writeFile;
		long _readFileSize, _writeFileSize;
		// When writing, this is the time the file was opened. When reading, it's the reset time.
		struct timeval _openTime;
		unsigned int _debug;
		size_t _readUsage, _writeUsage;
		size_t _readSize, _writeSize;
		uint8_t *_readBuffer, *_writeBuffer;
		bool _ignoreTimes;

		void AllocateReadBuffer (unsigned int size = 0);
		void AllocateWriteBuffer (unsigned int size = 0);
		void DeallocateReadBuffer ();
		void DeallocateWriteBuffer ();

		void GetCurrentFileTime (struct timeval &dest);
		bool DataAvailableWithinLimit (FILE * const file, const struct timeval &limit);
		void GetNextChunkInfo (FILE * const file, struct timeval &timeStamp, size_t &size);
		size_t GetChunksToTimeLimit (FILE * const file, void *data, size_t count,
								const struct timeval &limit);
		size_t GetChunkSizesToTimeLimit (FILE * const file, const struct timeval &limit);
		size_t GetSingleChunk (FILE * const file, void *data, size_t count,
								struct timeval &timeStamp, size_t &size);
		size_t GetFileSize (FILE * const file);

		void ReadFromFile (FILE * const file, void * const dest, size_t count);
		void WriteToFile (FILE * const file, const void * const data, size_t count);
		void WriteTimeStamp (FILE * const file);
};

} // namespace flexiport

#endif // __LOGFILE_H
