#include <hokuyo_aist.h>
using namespace hokuyo_aist;

#include <boost/python.hpp>
#include <boost/python/def.hpp>

#include <iostream>

class BaseErrorWrap
    : public BaseError, public boost::python::wrapper<BaseError>
{
    public:
        BaseErrorWrap(unsigned int desc_code, char const* error_type)
            : BaseError(desc_code, error_type)
        {}

        BaseErrorWrap(unsigned int desc_code, std::string error_type)
            : BaseError(desc_code, error_type.c_str())
        {}

        unsigned int desc_code() const throw()
        {
            if (boost::python::override f = get_override("desc_code"))
            {
                return f();
            }
            return BaseError::desc_code();
        }
        unsigned int default_desc_code() const throw()
        {
            return BaseError::desc_code();
        }

        char const* error_type() const throw()
        {
            if (boost::python::override f = get_override("error_type"))
            {
                return f();
            }
            return BaseError::error_type();
        }
        char const* default_error_type() const throw()
        {
            return BaseError::error_type();
        }

        const char* what() throw()
        {
            if (boost::python::override f = get_override("what"))
            {
                return f();
            }
            return BaseError::what();
        }
        const char* default_what() throw()
        {
            return BaseError::what();
        }
};


class ScanDataWrap : public ScanData, public boost::python::wrapper<ScanData>
{
    public:
        ScanDataWrap(void)
            : ScanData()
        {}
        ScanDataWrap(ScanDataWrap const& rhs)
            : ScanData(rhs)
        {}

        uint32_t range(unsigned int index)
            { return ranges_[index]; }
        /*unsigned int ranges_length() const
        {
            if (boost::python::override f = get_override("ranges_length"))
            {
                return f();
            }
            return ScanData::ranges_length();
        }*/

        uint32_t intensity(unsigned int index)
            { return intensities_[index]; }
        /*unsigned int intensities_length() const
        {
            if (boost::python::override f = get_override("intensities_length"))
            {
                return f();
            }
            return ScanData::intensities_length();
        }*/

        /*bool get_error_status() const
        {
            if (boost::python::override f = get_override("get_error_status"))
            {
                return f();
            }
            return ScanData::get_error_status();
        }

        std::string error_code_to_string(uint32_t error_code)
        {
            if (boost::python::override f = get_override("error_code_to_string"))
            {
                return f(error_code);
            }
            return ScanData::error_code_to_string(error_code);
        }

        unsigned int laser_time_stamp()
        {
            if (boost::python::override f = get_override("laser_time_stamp"))
            {
                return f();
            }
            return ScanData::laser_time_stamp();
        }

        unsigned int system_time_stamp()
        {
            if (boost::python::override f = get_override("system_time_stamp"))
            {
                return f();
            }
            return ScanData::system_time_stamp();
        }*/
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads1,
        get_ranges, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads2,
        get_ranges_by_angle, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads3,
        get_ranges_intensities, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads4,
        get_ranges_intensities_by_angle, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads5,
        get_new_ranges, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads6,
        get_new_ranges_by_angle, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads7,
        get_new_ranges_intensities, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_overloads8,
        get_new_ranges_intensities_by_angle, 3, 4)

BOOST_PYTHON_MODULE(hokuyo_aist)
{
    using namespace boost::python;

    class_<BaseErrorWrap, boost::noncopyable>("BaseError", init<unsigned int, std::string>())
        .def("desc_code", &BaseError::desc_code, &BaseErrorWrap::default_desc_code)
        .def("error_type", &BaseError::error_type, &BaseErrorWrap::default_error_type)
        .def("what", &BaseError::what, &BaseErrorWrap::default_what)
        ;

    class_<ScanDataWrap, boost::noncopyable>("ScanData")
        // TODO: write a wrapper function to copy the data into a python array, because this doesn't work
//        .def("ranges", &ScanData::ranges, return_value_policy<reference_existing_object> (), with_custodian_and_ward_postcall<1, 0> ())
//        .def("intensities", &ScanData::intensities, return_value_policy<reference_existing_object> (), with_custodian_and_ward_postcall<1, 0> ())
        .def("range", &ScanDataWrap::range)
        .def("intensity", &ScanDataWrap::intensity)
        .def("ranges_length", &ScanData::ranges_length)
        .def("intensities_length", &ScanData::intensities_length)
        .def("get_error_status", &ScanData::get_error_status)
        .def("error_code_to_string", &ScanData::error_code_to_string)
        .def("laser_time_stamp", &ScanData::laser_time_stamp)
        .def("system_time_stamp", &ScanData::system_time_stamp)
        .def("model", &ScanData::model)
        .def("buffers_provided", &ScanData::buffers_provided)
        .def("as_string", &ScanData::as_string)
        .def("clean_up", &ScanData::clean_up)
        ;

    class_<Sensor>("Sensor")
        .def("open", &Sensor::open)
        .def("open_with_probing", &Sensor::open_with_probing)
        .def("close", &Sensor::close)
        .def("is_open", &Sensor::is_open)
        .def("set_power", &Sensor::set_power)
        .def("set_baud", &Sensor::set_baud)
        .def("set_ip", &Sensor::set_ip)
        .def("reset", &Sensor::reset)
        .def("semi_reset", &Sensor::semi_reset)
        .def("set_motor_speed", &Sensor::set_motor_speed)
        .def("set_high_sensitivity", &Sensor::set_high_sensitivity)
        .def("get_sensor_info", &Sensor::get_sensor_info)
        .def("get_time", &Sensor::get_time)
        .def("get_raw_time", &Sensor::get_raw_time)
        .def("calibrate_time", &Sensor::calibrate_time)
        .def("time_offset", &Sensor::time_offset)
        .def("set_time_offset", &Sensor::set_time_offset)
        .def("drift_rate", &Sensor::drift_rate)
        .def("set_drift_rate", &Sensor::set_drift_rate)
        .def("skew_alpha", &Sensor::skew_alpha)
        .def("set_skew_alpha", &Sensor::set_skew_alpha)
        .def("get_ranges", &Sensor::get_ranges, sensor_overloads1())
        .def("get_ranges_by_angle",
                &Sensor::get_ranges_by_angle, sensor_overloads2())
        .def("get_ranges_intensities",
                &Sensor::get_ranges_intensities, sensor_overloads3())
        .def("get_ranges_intensities_by_angle",
                &Sensor::get_ranges_intensities_by_angle, sensor_overloads4())
        .def("get_new_ranges",
                &Sensor::get_new_ranges, sensor_overloads5())
        .def("get_new_ranges_by_angle",
                &Sensor::get_new_ranges_by_angle, sensor_overloads6())
        .def("get_new_ranges_intensities",
                &Sensor::get_new_ranges_intensities, sensor_overloads7())
        .def("get_new_ranges_intensities_by_angle",
                &Sensor::get_new_ranges_intensities_by_angle,
                sensor_overloads8())
        .def("scip_version", &Sensor::scip_version)
        .def("set_verbose", &Sensor::set_verbose)
        .def("ignore_unknowns", &Sensor::ignore_unknowns)
        .def("set_multiecho_mode", &Sensor::set_multiecho_mode)
        .def("step_to_angle", &Sensor::step_to_angle)
        .def("angle_to_step", &Sensor::angle_to_step)
        ;
}
