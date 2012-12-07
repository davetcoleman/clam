#!/usr/bin/env python

import flexiport, sys, time

def Usage ():
	print 'Usage: ' + sys.argv[0] + ' [-o portoptions]'
	print '-o portoptions\tString of options for the port.'


def main ():
	portOptions = 'type=serial,device=/dev/ttyS0,timeout=1'

	if len (sys.argv) == 3:
		if sys.argv[1] == '-o':
			portOptions = sys.argv[2]
		else:
			Usage ()
			sys.exit (1)
	elif len (sys.argv) != 1:
		Usage ()
		sys.exit (1)

	try:
		port = flexiport.CreatePort (portOptions)
		port.Open ()
		port.Flush ()
		print port.GetStatus ()

		print 'Writing data to port, should read same data back.'
		#stringMessage = 'Message #1'
		#print 'Writing "' + stringMessage + '" using WriteString()'
		#if port.WriteString (stringMessage) != len (stringMessage):
			#print 'Test failed: did not write enough bytes.'
			#sys.exit (1)
		#time.sleep (0.1)
		#print 'Testing ReadString ()'
		#if port.ReadString (stringBuffer) < 0:
			#print 'Timeout: Test failed.'
			#sys.exit (1)
		#print 'Read back using ReadString(): "' + stringBuffer + '"'
		#if stringBuffer != stringMessage:
			#print 'Test failed.'
			#sys.exit (1)

		stringMessage = 'Message #2'
		print 'Writing "' + stringMessage + '" plus NULL using Write()'
		if port.Write (stringMessage, len (stringMessage) + 1) != len (stringMessage) + 1:
			print 'Test failed: did not write enough bytes.'
			sys.exit (1)
		print 'Testing ReadStringUntil()'
		if port.ReadStringUntil (stringBuffer, '\n') < 0:
			print 'Timeout: Test failed.'
			sys.exit (1)
		print 'Received \"' + stringBuffer + '"'
		if stringBuffer != stringMessage:
			print 'Test failed.'
			sys.exit (1)

		port.Close ()
	except flexiport.PortException, e:
		print 'Caught exception: ' + e.what ()
		sys.exit (1)


if __name__ == '__main__':
	main ()
