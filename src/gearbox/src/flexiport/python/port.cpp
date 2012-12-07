#include <port.h>
using namespace flexiport;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

class PortWrap: public Port, public boost::python::wrapper<Port>
{
	public:
		void Open (void) { get_override ("Open") (); }
		void Close (void) { get_override ("Close") (); }

		ssize_t Read (void * const buffer, size_t count)
			{ return get_override ("Read") (buffer, count); }
		ssize_t ReadFull (void * const buffer, size_t count)
			{ return get_override ("ReadFull") (buffer, count); }

		ssize_t ReadString (std::string &buffer)
		{
			if (boost::python::override f = get_override ("ReadString"))
				return f (buffer);
			return Port::ReadString (buffer);
		}
		ssize_t DefaultReadString (std::string &buffer)
			{ return Port::ReadString (buffer); }

		ssize_t ReadUntil (void * const buffer, size_t count, uint8_t terminator)
		{
			if (boost::python::override f = get_override ("ReadUntil"))
				return f (buffer, count, terminator);
			return Port::ReadUntil (buffer, count, terminator);
		}
		ssize_t DefaultReadUntil (void * const buffer, size_t count, uint8_t terminator)
			{ return Port::ReadUntil (buffer, count, terminator); }

		ssize_t ReadStringUntil (std::string &buffer, char terminator)
		{
			if (boost::python::override f = get_override ("ReadStringUntil"))
				return f (buffer, terminator);
			return Port::ReadStringUntil (buffer, terminator);
		}
		ssize_t DefaultReadStringUntil (std::string &buffer, char terminator)
			{ return Port::ReadStringUntil (buffer, terminator); }

		ssize_t ReadLine1 (char * const buffer, size_t count)
		{
			if (boost::python::override f = get_override ("ReadLine"))
				return f (buffer, count);
			return Port::ReadLine (buffer, count);
		}
		ssize_t DefaultReadLine1 (char * const buffer, size_t count)
			{ return Port::ReadLine (buffer, count); }

		ssize_t ReadLine2 (std::string &buffer)
		{
			if (boost::python::override f = get_override ("ReadLine"))
				return f (buffer);
			return Port::ReadLine (buffer);
		}
		ssize_t DefaultReadLine2 (std::string &buffer)
			{ return Port::ReadLine (buffer); }

		ssize_t Skip (size_t count)
		{
			if (boost::python::override f = get_override ("Skip"))
				return f (count);
			return Port::Skip (count);
		}
		ssize_t DefaultSkip (size_t count)
			{ return Port::Skip (count); }

		ssize_t SkipUntil (uint8_t terminator, unsigned int count)
		{
			if (boost::python::override f = get_override ("SkipUntil"))
				return f (terminator, count);
			return Port::SkipUntil (terminator, count);
		}
		ssize_t DefaultSkipUntil (uint8_t terminator, unsigned int count)
			{ return Port::SkipUntil (terminator, count); }

		ssize_t BytesAvailable (void) { return get_override ("BytesAvailable") (); }
		ssize_t BytesAvailableWait (void) { return get_override ("BytesAvailableWait") (); }
		ssize_t Write (const void * const buffer, size_t count)
			{ return get_override ("Write") (buffer, count); }

		ssize_t WriteFull (const void * const buffer, size_t count)
		{
			if (boost::python::override f = get_override ("WriteFull"))
				return f (buffer, count);
			return Port::WriteFull (buffer, count);
		}
		ssize_t DefaultWriteFull (const void * const buffer, size_t count)
			{ return Port::WriteFull (buffer, count); }

		ssize_t WriteString1 (const char * const buffer)
		{
			if (boost::python::override f = get_override ("WriteString"))
				return f (buffer);
			return Port::WriteString (buffer);
		}
		ssize_t DefaultWriteString1 (const char * const buffer)
			{ return Port::WriteString (buffer); }

		ssize_t WriteString2 (const std::string &buffer)
		{
			if (boost::python::override f = get_override ("WriteString"))
				return f (buffer);
			return Port::WriteString (buffer);
		}
		ssize_t DefaultWriteString2 (const std::string &buffer)
			{ return Port::WriteString (buffer); }

		void Flush (void) { get_override ("Flush") (); }
		void Drain (void) { get_override ("Drain") (); }

		std::string GetStatus (void) const
		{
			if (boost::python::override f = get_override ("GetStatus"))
				return f ();
			return Port::GetStatus ();
		}
		std::string DefaultGetStatus (void) const
			{ return Port::GetStatus (); }

		void SetTimeout (Timeout timeout) { get_override ("SetTimeout") (timeout); }
		Timeout GetTimeout (void) const
		{
			if (boost::python::override f = get_override ("GetTimeout"))
				return f ();
			return Port::GetTimeout ();
		}
		Timeout DefaultGetTimeout (void) const
			{ return Port::GetTimeout (); }

		void SetCanRead (bool canRead) { get_override ("SetCanRead") (canRead); }
		bool CanRead (void) const
		{
			if (boost::python::override f = get_override ("CanRead"))
				return f ();
			return Port::CanRead ();
		}
		bool DefaultCanRead (void) const
			{ return Port::CanRead (); }

		void SetCanWrite (bool canWrite) { get_override ("SetCanWrite") (canWrite); }
		bool CanWrite (void) const
		{
			if (boost::python::override f = get_override ("CanWrite"))
				return f ();
			return Port::CanWrite ();
		}
		bool DefaultCanWrite (void) const
			{ return Port::CanWrite (); }

		bool IsOpen (void) const { return get_override ("IsOpen") (); }
};

void PortClassDef (void)
{
	using namespace boost::python;

	class_<PortWrap, boost::noncopyable> ("Port", no_init)
		.def ("Open", pure_virtual (&Port::Open))
		.def ("Close", pure_virtual (&Port::Close))
		.def ("Read", pure_virtual (&Port::Read))
		.def ("ReadFull", pure_virtual (&Port::ReadFull))
		.def ("ReadString", &Port::ReadString, &PortWrap::DefaultReadString)
		.def ("ReadUntil", &Port::ReadUntil, &PortWrap::DefaultReadUntil)
		.def ("ReadStringUntil", &Port::ReadStringUntil, &PortWrap::ReadStringUntil)
		// The boost::python docs show how to do overloaded functions, and how to do virtual
		// functions, but not both at the same time... TODO: Figure this out later.
//		.def ("ReadLine", &PortWrap::ReadLine1, &PortWrap::DefaultReadLine1)
//		.def ("ReadLine", &PortWrap::ReadLine2, &PortWrap::DefaultReadLine2)
		.def ("Skip", &Port::Skip, &PortWrap::DefaultSkip)
		.def ("SkipUntil", &Port::SkipUntil, &PortWrap::DefaultSkipUntil)
		.def ("BytesAvailable", pure_virtual (&Port::BytesAvailable))
		.def ("BytesAvailableWait", pure_virtual (&Port::BytesAvailableWait))
		.def ("Write", pure_virtual (&Port::Write))
		.def ("WriteFull", &Port::WriteFull, &PortWrap::DefaultWriteFull)
//		.def ("WriteString", &PortWrap::WriteString1, &PortWrap::DefaultWriteString1)
//		.def ("WriteString", &PortWrap::WriteString2, &PortWrap::DefaultWriteString2)
		.def ("Flush", pure_virtual (&Port::Flush))
		.def ("Drain", pure_virtual (&Port::Drain))
		.def ("GetStatus", &Port::GetStatus, &PortWrap::DefaultGetStatus)
		.add_property ("portType", &Port::GetPortType)
		.add_property ("debug", &Port::GetDebug, &Port::SetDebug)
		.def ("GetTimeout", &Port::GetTimeout, &PortWrap::DefaultGetTimeout)
		.def ("IsBlocking", &Port::IsBlocking)
		.def ("SetCanRead", pure_virtual (&Port::SetCanRead))
		.def ("CanRead", &Port::CanRead, &PortWrap::DefaultCanRead)
		.def ("SetCanWrite", pure_virtual (&Port::SetCanWrite))
		.def ("CanWrite", &Port::CanWrite, &PortWrap::DefaultCanWrite)
		.def ("IsOpen", pure_virtual (&Port::IsOpen))
		;
}
