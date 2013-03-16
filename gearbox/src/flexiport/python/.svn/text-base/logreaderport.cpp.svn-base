#include <logreaderport.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

void LogReaderPortClassDef (void)
{
	using namespace boost::python;

	class_<LogReaderPort, bases<Port>, boost::noncopyable> ("LogReaderPort",
			init<std::map<std::string, std::string> > ())
		.def ("Open", &LogReaderPort::Open)
		.def ("Close", &LogReaderPort::Close)
		.def ("Read", &LogReaderPort::Read)
		.def ("ReadFull", &LogReaderPort::ReadFull)
		.def ("BytesAvailable", &LogReaderPort::BytesAvailable)
		.def ("BytesAvailableWait", &LogReaderPort::BytesAvailableWait)
		.def ("Write", &LogReaderPort::Write)
		.def ("Flush", &LogReaderPort::Flush)
		.def ("Drain", &LogReaderPort::Drain)
		.def ("GetStatus", &LogReaderPort::GetStatus)
		.def ("SetTimeout", &LogReaderPort::SetTimeout)
		.def ("SetCanRead", &LogReaderPort::SetCanRead)
		.def ("SetCanWrite", &LogReaderPort::SetCanWrite)
		.def ("IsOpen", &LogReaderPort::IsOpen)
		;
}
