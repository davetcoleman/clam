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

inline void SLEEP (int length)
{
#if defined (WIN32)
	Sleep (length * 1000);
#else
	sleep (length);
#endif
}

#if defined (WIN32)
DWORD WINAPI DoChild (LPVOID param)
{
	string portOptions = *(reinterpret_cast<string*> (param));
#else
int DoChild (string portOptions)
{
#endif
	cout << "Waiting a few seconds..." << endl;
	SLEEP (5);
	cout << "Creating client port" << endl;
	flexiport::Port *port;

	port = flexiport::CreatePort ("type=tcp," + portOptions);
	cout << port->GetStatus ();

	cout << "Client opening port." << endl;
	port->Open ();
	cout << "Client port open. Testing sending data to server." << endl;
	cout << "Client sending 'Message #1'" << endl;
	string stringMessage = "Message #1";
	port->WriteString (stringMessage);
	SLEEP (5);
	cout << "Client sending 'Message #2\\n'" << endl;
	char charMessage[] = "Message #2\n";
	port->Write (charMessage, strlen (charMessage) + 1);
	SLEEP (5);
	cout << "Client sending 'This is message #3'" << endl;
	port->WriteString ("This is message #3");
	SLEEP (5);
	cout << "Client sending 'Probably message #4\\n'" << endl;
	char charMessage2[] = "Probably message #4\n";
	port->Write (charMessage2, strlen (charMessage2) + 1);
	SLEEP (5);
	cout << "Client sending 'Daaaaa.'" << endl;
	port->WriteString ("Daaaaa.");
	SLEEP (5);
	cout << "Client sending 'Finally, message #6\\n'" << endl;
	char charMessage3[] = "Finally, message #6\n";
	port->Write (charMessage3, strlen (charMessage3) + 1);
	cout << "Client waiting for parting message." << endl;
	port->ReadString (stringMessage);
	cout << "Client got parting message: \"" << stringMessage << '"' << endl;
	cout << "Client done." << endl;

	return 0;
}

int DoParent (string portOptions)
{
	string stringBuffer;
	char charBuffer[32];
	memset (charBuffer, 0, sizeof (char) * 32);

	cout << "Creating listener port" << endl;
	flexiport::Port *port;

	port = flexiport::CreatePort ("type=tcp,listen," + portOptions);
	cout << port->GetStatus ();

	cout << "Server opening port." << endl;
	port->Open ();
	cout << "Server port is open." << endl;
	cout << "Server testing ReadString()" << endl;
	port->ReadString (stringBuffer);
	cout << "Server received \"" << stringBuffer << '"' << endl;
	if (stringBuffer != "Message #1")
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server testing ReadStringUntil()" << endl;
	port->ReadStringUntil (stringBuffer, '\n');
	cout << "Server received \"" << stringBuffer << '"' << endl;
	if (stringBuffer != "Message #2\n")
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server has " << port->BytesAvailable () << " bytes available immediatly." << endl;
	cout << "Server clearing the trailing null byte from that last message." << endl;
	port->Read (charBuffer, 1);
	int bytesWaiting = port->BytesAvailableWait ();
	cout << "Server has " << bytesWaiting << " bytes available after waiting." << endl;
	cout << "Server testing ReadFull()" << endl;
	port->ReadFull (charBuffer, bytesWaiting);
	cout << "Server received \"" << charBuffer << '"' << endl;
	if (strncmp (charBuffer, "This is message #3", 18) != 0)
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server testing ReadUntil()" << endl;
	port->ReadUntil (charBuffer, 32, '\n');
	cout << "Server received \"" << charBuffer << '"' << endl;
	if (strncmp (charBuffer, "Probably message #4\n", 20) != 0)
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server clearing the trailing null byte from that last message." << endl;
	port->Read (charBuffer, 1);
	cout << "Server testing Read()" << endl;
	memset (charBuffer, 0, sizeof (char) * 32);
	port->Read (charBuffer, 32);
	cout << "Server received \"" << charBuffer << '"' << endl;
	if (strncmp (charBuffer, "Daaaaa.", 7) != 0)
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server testing ReadLine()" << endl;
	port->ReadLine (stringBuffer);
	cout << "Server received \"" << stringBuffer << '"' << endl;
	if (stringBuffer != "Finally, message #6\n")
	{
		cout << "Test failed." << endl;
		return -1;
	}
	cout << "Server sending back to client" << endl;
	port->WriteString ("So long, and thanks for all the text.");
	SLEEP (5);

	return 0;
}

int main (int argc, char **argv)
{
	string portOptions;

#if defined (WIN32)
	portOptions = "";
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
				cout << "-o options\tString of extra options for the port." << endl;
				return 1;
		}
	}
#endif

	try
	{
		// Fork and create a listener and a connector process
#if defined (WIN32)
		if (!CreateThread (NULL, 0, DoChild, &portOptions, 0, NULL))
		{
			cerr << "Failed to create child thread." << endl;
			return 1;
		}
		if (DoParent (portOptions) < 0)
			return 1;
#else
		pid_t result;
		result = fork ();
		if (result < 0)
		{
			cerr << "Failed to fork: (" << errno << ") " << strerror (errno) << endl;
			return -1;
		}
		else if (result == 0)
		{
			if (DoChild (portOptions) < 0)
				return 1;
		}
		else
		{
			if (DoParent (portOptions) < 0)
				return 1;
		}
#endif
	}
	catch (flexiport::PortException e)
	{
		cerr << "Caught exception: " << e.what () << endl;
		return 1;
	}

	return 0;
}
