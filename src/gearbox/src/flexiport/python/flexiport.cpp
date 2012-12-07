#include <flexiport.h>
#include <port.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

void TimeoutClassDef (void);
void PortClassDef (void);
void SerialPortClassDef (void);
void TCPPortClassDef (void);
void LogWriterPortClassDef (void);
void LogReaderPortClassDef (void);

class PortExceptionWrap : public PortException, public boost::python::wrapper<PortException>
{
	public:
		const char* what (void) const throw ()
		{
			if (boost::python::override f = get_override ("what"))
				return f ();
			return PortException::what ();
		}
		const char* Defaultwhat (void) const throw ()
			{ return PortException::what (); }
};

Port* (*CreatePort1) (std::map<std::string, std::string>) = &CreatePort;
Port* (*CreatePort2) (std::string) = &CreatePort;

BOOST_PYTHON_MODULE (flexiport)
{
	using namespace boost::python;

	class_<PortException> ("PortException", init<const char*> ())
		.def (init<const std::string&> ())
		.def ("what", &PortException::what, &PortExceptionWrap::Defaultwhat)
		;

	def ("CreatePort", CreatePort1, return_value_policy<manage_new_object> ());
	def ("CreatePort", CreatePort2, return_value_policy<manage_new_object> ());

	TimeoutClassDef ();
	PortClassDef ();
	SerialPortClassDef ();
	TCPPortClassDef ();
	LogWriterPortClassDef ();
	LogReaderPortClassDef ();
}
