#include <serialport.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

void SerialPortClassDef (void)
{
	using namespace boost::python;

	class_<SerialPort, bases<Port>, boost::noncopyable> ("SerialPort",
			init<std::map<std::string, std::string> > ())
		.def ("Open", &SerialPort::Open)
		.def ("Close", &SerialPort::Close)
		.def ("Read", &SerialPort::Read)
		.def ("ReadFull", &SerialPort::ReadFull)
		.def ("BytesAvailable", &SerialPort::BytesAvailable)
		.def ("BytesAvailableWait", &SerialPort::BytesAvailableWait)
		.def ("Write", &SerialPort::Write)
		.def ("Flush", &SerialPort::Flush)
		.def ("Drain", &SerialPort::Drain)
		.def ("GetStatus", &SerialPort::GetStatus)
		.def ("SetTimeout", &SerialPort::SetTimeout)
		.def ("SetCanRead", &SerialPort::SetCanRead)
		.def ("SetCanWrite", &SerialPort::SetCanWrite)
		.def ("IsOpen", &SerialPort::IsOpen)
		.def ("SetBaudRate", &SerialPort::SetBaudRate)
		.def ("GetBaudRate", &SerialPort::GetBaudRate)
		;
}
