#include <logwriterport.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

void LogWriterPortClassDef (void)
{
	using namespace boost::python;

	class_<LogWriterPort, bases<Port>, boost::noncopyable> ("LogWriterPort",
			init<std::map<std::string, std::string> > ())
		.def ("Open", &LogWriterPort::Open)
		.def ("Close", &LogWriterPort::Close)
		.def ("Read", &LogWriterPort::Read)
		.def ("ReadFull", &LogWriterPort::ReadFull)
		.def ("Skip", &LogWriterPort::Skip)
		.def ("SkipUntil", &LogWriterPort::SkipUntil)
		.def ("BytesAvailable", &LogWriterPort::BytesAvailable)
		.def ("BytesAvailableWait", &LogWriterPort::BytesAvailableWait)
		.def ("Write", &LogWriterPort::Write)
		.def ("Flush", &LogWriterPort::Flush)
		.def ("Drain", &LogWriterPort::Drain)
		.def ("GetStatus", &LogWriterPort::GetStatus)
		.def ("SetTimeout", &LogWriterPort::SetTimeout)
		.def ("SetCanRead", &LogWriterPort::SetCanRead)
		.def ("SetCanWrite", &LogWriterPort::SetCanWrite)
		.def ("IsOpen", &LogWriterPort::IsOpen)
		;
}
