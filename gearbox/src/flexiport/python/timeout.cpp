#include <boost/python.hpp>
#include <boost/python/def.hpp>

#include <timeout.h>
using namespace flexiport;

void TimeoutClassDef (void)
{
	using namespace boost::python;

	class_<Timeout> ("Timeout", init<int, int> ())
		.def (init<const Timeout&> ())

		.def ("AsTimeval", &Timeout::AsTimeval)
		.def ("FromTimeval", &Timeout::FromTimeval)
		.def ("AsTimespec", &Timeout::AsTimespec)
		.def ("FromTimespec", &Timeout::FromTimespec)

		.def_readwrite ("_sec", &Timeout::_sec)
		.def_readwrite ("_usec", &Timeout::_usec)
	;
}
