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

#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <sstream>
using namespace std;

#include <flexiport/flexiport.h>
#include <flexiport/port.h>
using namespace flexiport;

void Usage (char *progName)
{
	cout << "Usage: " << progName << " [options]" << endl << endl;
	cout << "-b size\t\tBuffer size. The maximum quantity of data that can be moved at once." << endl;
	cout << "\t\tIf set to 0, the buffer will shrink and grow as necessary based on" << endl;
	cout << "\t\tthe data waiting, but this will introduce delays." << endl;
	cout << "-l options\tString of extra options for the left-side port." << endl;
	cout << "-r options\tString of extra options for the right-side port." << endl;
	cout << "-s time\t\tSleep time between port checks (in microseconds)." << endl;
	cout << "\t\tSet to 0 for no sleep (the default)." << endl;
	cout << "-v\t\tVerbose mode." << endl;
}

int main (int argc, char **argv)
{
	Port *leftPort = NULL, *rightPort = NULL;
	int opt;
	char c;
	string leftPortOptions, rightPortOptions;
	bool verbose = false;
	unsigned int bufferSize = 0, sleepTime = 0;
	uint8_t *buffer = NULL;
	istringstream is;

	// Get some options from the command line
	while ((opt = getopt (argc, argv, "b:hl:r:s:v")) != -1)
	{
		switch (opt)
		{
			case 'b':
				is.str (optarg);
				if (!(is >> bufferSize) || is.get (c) || bufferSize < 0)
				{
					cerr << "Bad buffer size: " << optarg << endl;
					Usage (argv[0]);
					exit (1);
				}
				break;
			case 'h':
				Usage (argv[0]);
				exit (1);
				break;
			case 'l':
				leftPortOptions = optarg;
				break;
			case 'r':
				rightPortOptions = optarg;
				break;
			case 's':
				is.str (optarg);
				if (!(is >> sleepTime) || is.get (c) || sleepTime < 0)
				{
					cerr << "Bad sleep time: " << optarg << endl;
					Usage (argv[0]);
					exit (1);
				}
				break;
			case 'v':
				verbose = true;
				break;
			default:
				Usage (argv[0]);
				exit (1);
		}
	}
	
	try
	{
		if (verbose)
			cerr << "Creating ports." << endl;
		leftPort = CreatePort (leftPortOptions);
		rightPort = CreatePort (rightPortOptions);

		if (!leftPort->IsOpen ())
		{
			if (verbose)
				cerr << "Opening left port." << endl;
			leftPort->Open ();
		}
		if (!rightPort->IsOpen ())
		{
			if (verbose)
				cerr << "Opening right port." << endl;
			rightPort->Open ();
		}

		if (verbose)
		{
			if (bufferSize > 0)
				cerr << "Allocating buffer of size " << bufferSize << " bytes." << endl;
			else
				cerr << "Using dynamic buffer." << endl;
		}
		if (bufferSize > 0)
		{
			if ((buffer = reinterpret_cast<uint8_t*> (malloc (sizeof (uint8_t) *
					bufferSize))) == NULL)
			{
				cerr << "Failed to allocate memory for buffer." << endl;
				exit (1);
			}
		}
		else
			buffer = NULL;

		// Begin an infinite loop, checking each port for data and then sleeping a little while
		while (true)
		{
			int bytesWaiting = 0, bytesToRead = 0, bytesRead = 0, bytesWritten = 0;

			// Check the left port for waiting data
			if (verbose)
				cerr << "Checking left port for data." << endl;
			bytesWaiting = leftPort->BytesAvailableWait ();
			if (bytesWaiting > 0)
			{
				if (verbose)
				{
					cerr << "There are " << bytesWaiting << 
						" bytes waiting on the left." << endl;
				}
				if (bufferSize == 0)
				{
					// Allocate space
					if ((buffer = reinterpret_cast<uint8_t*> (realloc (buffer,
							sizeof (uint8_t) * bytesWaiting + 1))) == NULL)
					{
						cerr << "Failed to reallocate memory for buffer." << endl;
						exit (1);
					}
					bytesToRead = bytesWaiting;
				}
				else
					bytesToRead = bufferSize;   // Read up to as much as we can fit
				// Read data from the left
				bytesRead = leftPort->Read (buffer, bytesToRead);
				buffer[bytesRead] = '\0';
				if (verbose)
					cerr << "Read " << bytesRead << " bytes from the left: |" << buffer << "|." << endl;
				if (bytesRead <= 0)
				{
					cerr << "Expected " << bytesToRead << 
						" bytes from the left, didn't get any." << endl;
				}
				else
				{
					// Write the data to the right port
					bytesWritten = rightPort->WriteFull (buffer, bytesRead);
					if (verbose)
						cerr << "Wrote " << bytesWritten << " bytes to the right." << endl;
				}
			}

			// Repeat the process with the right port
			if (verbose)
				cerr << "Checking right port for data." << endl;
			bytesWaiting = rightPort->BytesAvailableWait ();
			if (bytesWaiting > 0)
			{
				if (verbose)
				{
					cerr << "There are " << bytesWaiting << 
						" bytes waiting on the right." << endl;
				}
				if (bufferSize == 0)
				{
					// Allocate space
					if ((buffer = reinterpret_cast<uint8_t*> (realloc (buffer,
							sizeof (uint8_t) * bytesWaiting + 1))) == NULL)
					{
						cerr << "Failed to reallocate memory for buffer." << endl;
						exit (1);
					}
					bytesToRead = bytesWaiting;
				}
				else
					bytesToRead = bufferSize;   // Read up to as much as we can fit
				// Read data from the right
				bytesRead = rightPort->Read (buffer, bytesToRead);
				buffer[bytesRead] = '\0';
				if (verbose)
					cerr << "Read " << bytesRead << " bytes from the right: |" << buffer << "|." << endl;
				if (bytesRead <= 0)
				{
					cerr << "Expected " << bytesToRead << 
						" bytes from the right, didn't get any." << endl;
				}
				else
				{
					// Write the data to the left port
					bytesWritten = leftPort->WriteFull (buffer, bytesRead);
					if (verbose)
						cerr << "Wrote " << bytesWritten << " bytes to the left." << endl;
				}
			}

			// Sleep if set to do so
			if (sleepTime > 0)
			{
				if (verbose)
					cerr << "Sleeping for " << sleepTime << " microseconds." << endl;
				usleep (sleepTime);
			}
		}

		free (buffer);
		delete leftPort;
		delete rightPort;
	}
	catch (flexiport::PortException e)
	{
		cerr << "Caught exception: " << e.what () << endl;
		return -1;
	}
	
	return 0;
}
