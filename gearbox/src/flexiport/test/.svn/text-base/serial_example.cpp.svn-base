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

#ifndef WIN32
	#include <unistd.h>
#endif

#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <string>
#include <iostream>
using namespace std;

#include <flexiport/flexiport.h>
#include <flexiport/port.h>

#if defined (WIN32)
	#include <Windows.h>
#endif

inline void USleep (int length)
{
#if defined (WIN32)
	Sleep (length / 1000);
#else
	usleep (length);
#endif
}

int main (int argc, char **argv)
{
	string portOptions = "type=serial,device=/dev/ttyS0,timeout=1";

#if defined (WIN32)
	portOptions = "type=serial,device=COM1,timeout=1";
#else
	int opt;
	// Get some options from the command line
	while ((opt = getopt (argc, argv, "o:")) != -1)
	{
		switch (opt)
		{
			case 'o':
				portOptions = optarg;
				break;
			default:
				cout << "Usage: " << argv[0] << " [-o portoptions]" << endl << endl;
				cout << "-o options\tString of options for the port." << endl;
				return 1;
		}
	}
#endif

	try
	{
		string stringBuffer = "";
		char charBuffer[32];
		memset (charBuffer, 0, sizeof (char) * 32);

		cout << "Creating port" << endl;
		flexiport::Port *port;

		port = flexiport::CreatePort (portOptions);

		cout << "Opening port." << endl;
		port->Open ();
		port->Flush ();
		cout << port->GetStatus ();

		cout << "Writing data to port, should read same data back." << endl;
		string stringMessage = "Message #1";
		cout << "Writing 'Message #1' using WriteString()" << endl;
		if (port->WriteString (stringMessage) != static_cast<int> (stringMessage.size ()))
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		USleep (100000);
		cout << "Testing ReadString()" << endl;
		if (port->ReadString (stringBuffer) < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Read back using ReadString(): \"" << stringBuffer << '"' << endl;
		if (stringBuffer != stringMessage)
		{
			cout << "Test failed." << endl;
			return 1;
		}

		cout << "Writing 'Message #2\\n' plus NULL using Write()" << endl;
		char charMessage[] = "Message #2\n";
		if (port->Write (charMessage, strlen (charMessage) + 1) != static_cast<int> (strlen (charMessage) + 1))
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		cout << "Testing ReadStringUntil()" << endl;
		if (port->ReadStringUntil (stringBuffer, '\n') < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Received \"" << stringBuffer << '"' << endl;
		if (stringBuffer != charMessage)
		{
			cout << "Test failed." << endl;
			return 1;
		}

		cout << "There are " << port->BytesAvailable () << " bytes available immediatly." << endl;

		char charMessage2[] = "This is message #3";
		cout << "Writing '" << charMessage2 << "' using WriteString ()" << endl;
		if (port->WriteString (charMessage2) != static_cast<int> (strlen (charMessage2)))
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		cout << "Clearing the trailing null byte from the previous message." << endl;
		if (port->Read (charBuffer, 1) < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}

		cout << "Testing BytesAvailableWait()" << endl;
		int bytesWaiting = port->BytesAvailableWait ();
		cout << "There are " << bytesWaiting << " bytes available after waiting." << endl;

		cout << "Testing ReadFull()" << endl;
		if (port->ReadFull (charBuffer, strlen (charMessage2)) < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Received \"" << charBuffer << '"' << endl;
		if (strncmp (charBuffer, charMessage2, 32) != 0)
		{
			cout << "Test failed." << endl;
			return 1;
		}

		cout << "Writing 'Probably message #4\\n' using Write()" << endl;
		char charMessage3[] = "Probably message #4\n";
		if (port->Write (charMessage3, strlen (charMessage3)) != static_cast<int> (strlen (charMessage3)))
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		cout << "Testing ReadUntil()" << endl;
		if (port->ReadUntil (charBuffer, 32, '\n') < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Received \"" << charBuffer << '"' << endl;
		if (strncmp (charBuffer, charMessage3, 32) != 0)
		{
			cout << "Test failed." << endl;
			return 1;
		}

		cout << "Writing 'Daaaaa.' using WriteString()" << endl;
		if (port->WriteString ("Daaaaa.") != 7)
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		cout << "Adding a NULL to the end using Write()" << endl;
		char null = '\0';
		if (port->Write (&null, 1) != 1)
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		USleep (100000);
		cout << "Testing Read()" << endl;
		if (port->Read (charBuffer, 32) < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Received \"" << charBuffer << '"' << endl;
		if (strncmp (charBuffer, "Daaaaa.", 32) != 0)
		{
			cout << "Test failed." << endl;
			return 1;
		}

		cout << "Writing 'Finally, message #6\\n'" << endl;
		char charMessage4[] = "Finally, message #6\n";
		if (port->Write (charMessage4, strlen (charMessage4)) != static_cast<int> (strlen (charMessage4)))
		{
			cout << "Test failed: did not write enough bytes." << endl;
			return 1;
		}
		cout << "Testing ReadLine()" << endl;
		if (port->ReadLine (stringBuffer) < 0)
		{
			cout << "Timeout: Test failed." << endl;
			return 1;
		}
		cout << "Received \"" << stringBuffer << '"' << endl;
		if (stringBuffer != charMessage4)
		{
			cout << "Test failed." << endl;
			return 1;
		}
	}
	catch (flexiport::PortException e)
	{
		cerr << "Caught exception: " << e.what () << endl;
		return 1;
	}

	return 0;
}
