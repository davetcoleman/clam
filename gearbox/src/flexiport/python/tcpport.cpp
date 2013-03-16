#include <tcpport.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

void TCPPortClassDef (void)
{
	using namespace boost::python;

	class_<TCPPort, bases<Port>, boost::noncopyable> ("TCPPort",
			init<std::map<std::string, std::string> > ())
		.def ("Open", &TCPPort::Open)
		.def ("Close", &TCPPort::Close)
		.def ("Read", &TCPPort::Read)
		.def ("ReadFull", &TCPPort::ReadFull)
		.def ("BytesAvailable", &TCPPort::BytesAvailable)
		.def ("BytesAvailableWait", &TCPPort::BytesAvailableWait)
		.def ("Write", &TCPPort::Write)
		.def ("Flush", &TCPPort::Flush)
		.def ("Drain", &TCPPort::Drain)
		.def ("GetStatus", &TCPPort::GetStatus)
		.def ("SetTimeout", &TCPPort::SetTimeout)
		.def ("SetCanRead", &TCPPort::SetCanRead)
		.def ("SetCanWrite", &TCPPort::SetCanWrite)
		.def ("IsOpen", &TCPPort::IsOpen)
		;
}
