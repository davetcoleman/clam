#include <dynamixel_io.h>
#include <dynamixel_const.h>
#include <boost/python.hpp>
#include <boost/python/def.hpp>

using namespace dynamixel_hardware_interface;
using namespace boost::python;

template <class T>
std::vector<std::vector<T> > nestedSequenceToNestedVector(object t)
{
    std::vector<std::vector<T> > result;
    
    for (int i = 0; i < len(t); ++i)
    {
        std::vector<T> v;
        object mvals = t[i];
        
        for (int j = 0; j < len(mvals); ++j)
        {
            v.push_back(extract<T>(mvals[j]));
        }
        
        result.push_back(v);
    }
    
    return result;
}

std::vector<uint8_t> sequenceToVector(object l)
{
    std::vector<uint8_t> result;
    
    for (int i = 0; i < len(l); ++i)
    {
        result.push_back(extract<uint8_t>(l[i]));
    }
    
    return result;
}

list vectorToPyList(const std::vector<uint8_t>& vec)
{
    list result;
    
    for (int i = 0; i < vec.size(); ++i)
    {
        result.append(vec[i]);
    }
    
    return result;
}

class DynamixelIOWrap : public DynamixelIO
{
public:
    DynamixelIOWrap(std::string device, int baud) : DynamixelIO(device, boost::lexical_cast<std::string>(baud)) {}
    
    object getModelNumber(int servo_id)
    {
        uint16_t model;
        if (!DynamixelIO::getModelNumber(servo_id, model)) { return object(); }
        else { return object(model); }
    }
    
    object getFirmwareVersion(int servo_id)
    {
        uint8_t fw_ver;
        if (!DynamixelIO::getFirmwareVersion(servo_id, fw_ver)) { return object(); }
        else { return object(fw_ver); }
    }

    object getBaudRate(int servo_id)
    {
        uint8_t baud_rate;
        if (!DynamixelIO::getBaudRate(servo_id, baud_rate)) { return object(); }
        else { return object(baud_rate); }
    }
    
    object getReturnDelayTime(int servo_id)
    {
        uint8_t return_delay_time;
        if (!DynamixelIO::getReturnDelayTime(servo_id, return_delay_time)) { return object(); }
        else { return object(return_delay_time); }
    }
    
    object getAngleLimits(int servo_id)
    {
        uint16_t cw_angle_limit;
        uint16_t ccw_angle_limit;
        if (!DynamixelIO::getAngleLimits(servo_id, cw_angle_limit, ccw_angle_limit)) { return object(); }
        else { return make_tuple(cw_angle_limit, ccw_angle_limit); }
    }
    
    object getCWAngleLimit(int servo_id)
    {
        uint16_t cw_angle_limit;
        if (!DynamixelIO::getCWAngleLimit(servo_id, cw_angle_limit)) { return object(); }
        else { return object(cw_angle_limit); }
    }
    
    object getCCWAngleLimit(int servo_id)
    {
        uint16_t ccw_angle_limit;
        if (!DynamixelIO::getCCWAngleLimit(servo_id, ccw_angle_limit)) { return object(); }
        else { return object(ccw_angle_limit); }
    }
    
    object getVoltageLimits(int servo_id)
    {
        float min_voltage_limit;
        float max_voltage_limit;
        if (!DynamixelIO::getVoltageLimits(servo_id, min_voltage_limit, max_voltage_limit)) { return object(); }
        else { return make_tuple(min_voltage_limit, max_voltage_limit); }
    }
    
    object getMinVoltageLimit(int servo_id)
    {
        float min_voltage_limit;
        if (!DynamixelIO::getMinVoltageLimit(servo_id, min_voltage_limit)) { return object(); }
        else { return object(min_voltage_limit); }
    }
    
    object getMaxVoltageLimit(int servo_id)
    {
        float max_voltage_limit;
        if (!DynamixelIO::getMaxVoltageLimit(servo_id, max_voltage_limit)) { return object(); }
        else { return object(max_voltage_limit); }
    }
    
    object getTemperatureLimit(int servo_id)
    {
        uint8_t max_temperature;
        if (!DynamixelIO::getTemperatureLimit(servo_id, max_temperature)) { return object(); }
        else { return object(max_temperature); }
    }
    
    object getMaxTorque(int servo_id)
    {
        uint16_t max_torque;
        if (!DynamixelIO::getMaxTorque(servo_id, max_torque)) { return object(); }
        else { return object(max_torque); }
    }
    
    object getAlarmLed(int servo_id)
    {
        uint8_t alarm_led;
        if (!DynamixelIO::getAlarmLed(servo_id, alarm_led)) { return object(); }
        else { return object(alarm_led); }
    }
    
    object getAlarmShutdown(int servo_id)
    {
        uint8_t alarm_shutdown;
        if (!DynamixelIO::getAlarmShutdown(servo_id, alarm_shutdown)) { return object(); }
        else { return object(alarm_shutdown); }
    }

    object getTorqueEnable(int servo_id)
    {
        bool torque_enabled;
        if (!DynamixelIO::getTorqueEnable(servo_id, torque_enabled)) { return object(); }
        else { return object(torque_enabled); }
    }

    object getLedStatus(int servo_id)
    {
        bool led_enabled;
        if (!DynamixelIO::getLedStatus(servo_id, led_enabled)) { return object(); }
        else { return object(led_enabled); }
    }

    object getComplianceMargins(int servo_id)
    {
        uint8_t cw_compliance_margin;
        uint8_t ccw_compliance_margin;
        if (!DynamixelIO::getComplianceMargins(servo_id, cw_compliance_margin, ccw_compliance_margin)) { return object(); }
        else { return make_tuple(cw_compliance_margin, ccw_compliance_margin); }
    }
    
    object getCWComplianceMargin(int servo_id)
    {
        uint8_t cw_compliance_margin;
        if (!DynamixelIO::getCWComplianceMargin(servo_id, cw_compliance_margin)) { return object(); }
        else { return object(cw_compliance_margin); }
    }

    object getCCWComplianceMargin(int servo_id)
    {
        uint8_t ccw_compliance_margin;
        if (!DynamixelIO::getCCWComplianceMargin(servo_id, ccw_compliance_margin)) { return object(); }
        else { return object(ccw_compliance_margin); }
    }

    object getComplianceSlopes(int servo_id)
    {
        uint8_t cw_compliance_slope;
        uint8_t ccw_compliance_slope;
        if (!DynamixelIO::getComplianceSlopes(servo_id, cw_compliance_slope, ccw_compliance_slope)) { return object(); }
        else { return make_tuple(cw_compliance_slope, ccw_compliance_slope); }
    }
    
    object getCWComplianceSlope(int servo_id)
    {
        uint8_t cw_compliance_slope;
        if (!DynamixelIO::getCWComplianceSlope(servo_id, cw_compliance_slope)) { return object(); }
        else { return object(cw_compliance_slope); }
    }

    object getCCWComplianceSlope(int servo_id)
    {
        uint8_t ccw_compliance_slope;
        if (!DynamixelIO::getCCWComplianceSlope(servo_id, ccw_compliance_slope)) { return object(); }
        else { return object(ccw_compliance_slope); }
    }

    object getTargetPosition(int servo_id)
    {
        uint16_t target_position;
        if (!DynamixelIO::getTargetPosition(servo_id, target_position)) { return object(); }
        else { return object(target_position); }
    }

    object getTargetVelocity(int servo_id)
    {
        int16_t target_velocity;
        if (!DynamixelIO::getTargetVelocity(servo_id, target_velocity)) { return object(); }
        else { return object(target_velocity); }
    }

    object getTorqueLimit(int servo_id)
    {
        uint16_t target_limit;
        if (!DynamixelIO::getTorqueLimit(servo_id, target_limit)) { return object(); }
        else { return object(target_limit); }
    }

    object getPosition(int servo_id)
    {
        uint16_t position;
        if (!DynamixelIO::getPosition(servo_id, position)) { return object(); }
        else { return object(position); }
    }

    object getVelocity(int servo_id)
    {
        int16_t velocity;
        if (!DynamixelIO::getVelocity(servo_id, velocity)) { return object(); }
        else { return object(velocity); }
    }

    object getLoad(int servo_id)
    {
        int16_t load;
        if (!DynamixelIO::getLoad(servo_id, load)) { return object(); }
        else { return object(load); }
    }

    object getVoltage(int servo_id)
    {
        float voltage;
        if (!DynamixelIO::getVoltage(servo_id, voltage)) { return object(); }
        else { return object(voltage); }
    }

    object getTemperature(int servo_id)
    {
        uint8_t temperature;
        if (!DynamixelIO::getTemperature(servo_id, temperature)) { return object(); }
        else { return object(temperature); }
    }

    object getMoving(int servo_id)
    {
        bool is_moving;
        if (!DynamixelIO::getMoving(servo_id, is_moving)) { return object(); }
        else { return object(is_moving); }
    }

    object getFeedback(int servo_id)
    {
        DynamixelStatus status;
        
        if (!DynamixelIO::getFeedback(servo_id, status))
        {
            return object();
        }
        else
        {
            dict status_dict;
            
            status_dict["timestamp"] = status.timestamp;
            status_dict["torque_limit"] = status.torque_limit;
            status_dict["position"] = status.position;
            status_dict["velocity"] = status.velocity;
            status_dict["load"] = status.load;
            status_dict["voltage"] = status.voltage;
            status_dict["temperature"] = status.temperature;
            status_dict["moving"] = status.moving;
            
            return status_dict;
        }
    }
    
    bool setMultiPosition(object value_pairs)
    {
        return DynamixelIO::setMultiPosition(nestedSequenceToNestedVector<int>(value_pairs));
    }

    bool setMultiVelocity(object value_pairs)
    {
        return DynamixelIO::setMultiVelocity(nestedSequenceToNestedVector<int>(value_pairs));
    }

    bool setMultiPositionVelocity(object value_pairs)
    {
        return DynamixelIO::setMultiPositionVelocity(nestedSequenceToNestedVector<int>(value_pairs));
    }

    bool setMultiComplianceMargins(object value_pairs)
    {
        return DynamixelIO::setMultiComplianceMargins(nestedSequenceToNestedVector<int>(value_pairs));
    }
    
    bool setMultiComplianceSlopes(object value_pairs)
    {
        return DynamixelIO::setMultiComplianceSlopes(nestedSequenceToNestedVector<int>(value_pairs));
    }
    
    bool setMultiTorqueEnabled(object value_pairs)
    {
        return DynamixelIO::setMultiTorqueEnabled(nestedSequenceToNestedVector<int>(value_pairs));
    }
    
    bool setMultiTorqueLimit(object value_pairs)
    {
        return DynamixelIO::setMultiTorqueLimit(nestedSequenceToNestedVector<int>(value_pairs));
    }
    
    object read(int servo_id, DynamixelControl address, int size)
    {
        std::vector<uint8_t> response;
        if (!DynamixelIO::read(servo_id, address, size, response)) { return object(); }
        else { return vectorToPyList(response); }
    }

    object write(int servo_id, DynamixelControl address, object data)
    {
        std::vector<uint8_t> response;
        std::vector<uint8_t> datavec = sequenceToVector(data);
        
        if (!DynamixelIO::write(servo_id, address, datavec, response)) { return object(); }
        else { return vectorToPyList(response); }
    }

    bool syncWrite(DynamixelControl address, object data)
    {
        std::vector<std::vector<uint8_t> > datavec = nestedSequenceToNestedVector<uint8_t>(data);
        return DynamixelIO::syncWrite(address, datavec);
    }
};



BOOST_PYTHON_MODULE(dynamixel_io)
{
    // This will enable user-defined docstrings and python signatures,
    // while disabling the C++ signatures
    docstring_options local_docstring_options(true, true, false);

    def("get_motor_model_name", &getMotorModelName);

    def("get_motor_model_params", &getMotorModelParams);
    
    scope().attr("DXL_MAX_LOAD_ENCODER") = DXL_MAX_LOAD_ENCODER;
    scope().attr("DXL_MAX_VELOCITY_ENCODER") = DXL_MAX_VELOCITY_ENCODER;
    scope().attr("DXL_MAX_TORQUE_ENCODER") = DXL_MAX_TORQUE_ENCODER;

    enum_<DynamixelControl>("DynamixelControl")
        .value("DXL_MODEL_NUMBER_L", DXL_MODEL_NUMBER_L)
        .value("DXL_MODEL_NUMBER_H", DXL_MODEL_NUMBER_H)
        .value("DXL_FIRMWARE_VERSION", DXL_FIRMWARE_VERSION)
        .value("DXL_ID", DXL_ID)
        .value("DXL_BAUD_RATE", DXL_BAUD_RATE)
        .value("DXL_RETURN_DELAY_TIME", DXL_RETURN_DELAY_TIME)
        .value("DXL_CW_ANGLE_LIMIT_L", DXL_CW_ANGLE_LIMIT_L)
        .value("DXL_CW_ANGLE_LIMIT_H", DXL_CW_ANGLE_LIMIT_H)
        .value("DXL_CCW_ANGLE_LIMIT_L", DXL_CCW_ANGLE_LIMIT_L)
        .value("DXL_CCW_ANGLE_LIMIT_H", DXL_CCW_ANGLE_LIMIT_H)
        .value("DXL_DRIVE_MODE", DXL_DRIVE_MODE)
        .value("DXL_LIMIT_TEMPERATURE", DXL_LIMIT_TEMPERATURE)
        .value("DXL_DOWN_LIMIT_VOLTAGE", DXL_DOWN_LIMIT_VOLTAGE)
        .value("DXL_UP_LIMIT_VOLTAGE", DXL_UP_LIMIT_VOLTAGE)
        .value("DXL_MAX_TORQUE_L", DXL_MAX_TORQUE_L)
        .value("DXL_MAX_TORQUE_H", DXL_MAX_TORQUE_H)
        .value("DXL_RETURN_LEVEL", DXL_RETURN_LEVEL)
        .value("DXL_ALARM_LED", DXL_ALARM_LED)
        .value("DXL_ALARM_SHUTDOWN", DXL_ALARM_SHUTDOWN)
        .value("DXL_OPERATING_MODE", DXL_OPERATING_MODE)
        .value("DXL_DOWN_CALIBRATION_L", DXL_DOWN_CALIBRATION_L)
        .value("DXL_DOWN_CALIBRATION_H", DXL_DOWN_CALIBRATION_H)
        .value("DXL_UP_CALIBRATION_L", DXL_UP_CALIBRATION_L)
        .value("DXL_UP_CALIBRATION_H", DXL_UP_CALIBRATION_H)
        .value("DXL_TORQUE_ENABLE", DXL_TORQUE_ENABLE)
        .value("DXL_LED", DXL_LED)
        .value("DXL_CW_COMPLIANCE_MARGIN", DXL_CW_COMPLIANCE_MARGIN)
        .value("DXL_CCW_COMPLIANCE_MARGIN", DXL_CCW_COMPLIANCE_MARGIN)
        .value("DXL_CW_COMPLIANCE_SLOPE", DXL_CW_COMPLIANCE_SLOPE)
        .value("DXL_CCW_COMPLIANCE_SLOPE", DXL_CCW_COMPLIANCE_SLOPE)
        .value("DXL_GOAL_POSITION_L", DXL_GOAL_POSITION_L)
        .value("DXL_GOAL_POSITION_H", DXL_GOAL_POSITION_H)
        .value("DXL_GOAL_SPEED_L", DXL_GOAL_SPEED_L)
        .value("DXL_GOAL_SPEED_H", DXL_GOAL_SPEED_H)
        .value("DXL_TORQUE_LIMIT_L", DXL_TORQUE_LIMIT_L)
        .value("DXL_TORQUE_LIMIT_H", DXL_TORQUE_LIMIT_H)
        .value("DXL_PRESENT_POSITION_L", DXL_PRESENT_POSITION_L)
        .value("DXL_PRESENT_POSITION_H", DXL_PRESENT_POSITION_H)
        .value("DXL_PRESENT_SPEED_L", DXL_PRESENT_SPEED_L)
        .value("DXL_PRESENT_SPEED_H", DXL_PRESENT_SPEED_H)
        .value("DXL_PRESENT_LOAD_L", DXL_PRESENT_LOAD_L)
        .value("DXL_PRESENT_LOAD_H", DXL_PRESENT_LOAD_H)
        .value("DXL_PRESENT_VOLTAGE", DXL_PRESENT_VOLTAGE)
        .value("DXL_PRESENT_TEMPERATURE", DXL_PRESENT_TEMPERATURE)
        .value("DXL_REGISTERED_INSTRUCTION", DXL_REGISTERED_INSTRUCTION)
        .value("DXL_PAUSE_TIME", DXL_PAUSE_TIME)
        .value("DXL_MOVING", DXL_MOVING)
        .value("DXL_LOCK", DXL_LOCK)
        .value("DXL_PUNCH_L", DXL_PUNCH_L)
        .value("DXL_PUNCH_H", DXL_PUNCH_H)
        .value("DXL_SENSED_CURRENT_L", DXL_SENSED_CURRENT_L)
        .value("DXL_SENSED_CURRENT_H", DXL_SENSED_CURRENT_H)
        ;
    
    enum_<DynamixelInstruction>("DynamixelInstruction")
        .value("DXL_PING", DXL_PING)
        .value("DXL_READ_DATA", DXL_READ_DATA)
        .value("DXL_WRITE_DATA", DXL_WRITE_DATA)
        .value("DXL_REG_WRITE", DXL_REG_WRITE)
        .value("DXL_ACTION", DXL_ACTION)
        .value("DXL_RESET", DXL_RESET)
        .value("DXL_SYNC_WRITE", DXL_SYNC_WRITE)
        .value("DXL_BROADCAST", DXL_BROADCAST)
        ;
    
    enum_<DynamixelErrorCode>("DynamixelErrorCode")
        .value("DXL_INSTRUCTION_ERROR", DXL_INSTRUCTION_ERROR)
        .value("DXL_OVERLOAD_ERROR", DXL_OVERLOAD_ERROR)
        .value("DXL_CHECKSUM_ERROR", DXL_CHECKSUM_ERROR)
        .value("DXL_RANGE_ERROR", DXL_RANGE_ERROR)
        .value("DXL_OVERHEATING_ERROR", DXL_OVERHEATING_ERROR)
        .value("DXL_ANGLE_LIMIT_ERROR", DXL_ANGLE_LIMIT_ERROR)
        .value("DXL_INPUT_VOLTAGE_ERROR", DXL_INPUT_VOLTAGE_ERROR)
        .value("DXL_NO_ERROR", DXL_NO_ERROR)
        ;

    enum_<DynamixelParams>("DynamixelParams")
        .value("ENCODER_RESOLUTION", ENCODER_RESOLUTION)
        .value("RANGE_DEGREES", RANGE_DEGREES)
        .value("TORQUE_PER_VOLT", TORQUE_PER_VOLT)
        .value("VELOCITY_PER_VOLT", VELOCITY_PER_VOLT)
        ;
    
    class_<DynamixelData> ("DynamixelData")
        .def_readwrite("model_number", &DynamixelData::model_number)
        .def_readwrite("firmware_version", &DynamixelData::firmware_version)
        .def_readwrite("id", &DynamixelData::id)
        .def_readwrite("baud_rate", &DynamixelData::baud_rate)
        .def_readwrite("return_delay_time", &DynamixelData::return_delay_time)
        .def_readwrite("cw_angle_limit", &DynamixelData::cw_angle_limit)
        .def_readwrite("ccw_angle_limit", &DynamixelData::ccw_angle_limit)
        .def_readwrite("drive_mode", &DynamixelData::drive_mode)
        .def_readwrite("temperature_limit", &DynamixelData::temperature_limit)
        .def_readwrite("voltage_limit_low", &DynamixelData::voltage_limit_low)
        .def_readwrite("voltage_limit_high", &DynamixelData::voltage_limit_high)
        .def_readwrite("max_torque", &DynamixelData::max_torque)
        .def_readwrite("return_level", &DynamixelData::return_level)
        .def_readwrite("alarm_led", &DynamixelData::alarm_led)
        .def_readwrite("alarm_shutdown", &DynamixelData::alarm_shutdown)
        .def_readwrite("torque_enabled", &DynamixelData::torque_enabled)
        .def_readwrite("led", &DynamixelData::led)
        .def_readwrite("cw_compliance_margin", &DynamixelData::cw_compliance_margin)
        .def_readwrite("ccw_compliance_margin", &DynamixelData::ccw_compliance_margin)
        .def_readwrite("cw_compliance_slope", &DynamixelData::cw_compliance_slope)
        .def_readwrite("ccw_compliance_slope", &DynamixelData::ccw_compliance_slope)
        .def_readwrite("target_position", &DynamixelData::target_position)
        .def_readwrite("target_velocity", &DynamixelData::target_velocity)
        .def_readwrite("shutdown_error_time", &DynamixelData::shutdown_error_time)
        .def_readwrite("error", &DynamixelData::error)
        ;
    
    class_<DynamixelStatus> ("DynamixelStatus")
        .def_readwrite("timestamp", &DynamixelStatus::timestamp)
        .def_readwrite("torque_limit", &DynamixelStatus::torque_limit)
        .def_readwrite("position", &DynamixelStatus::position)
        .def_readwrite("velocity", &DynamixelStatus::velocity)
        .def_readwrite("load", &DynamixelStatus::load)
        .def_readwrite("voltage", &DynamixelStatus::voltage)
        .def_readwrite("temperature", &DynamixelStatus::temperature)
        .def_readwrite("moving", &DynamixelStatus::moving)
        ;
    
    class_<DynamixelIOWrap, boost::noncopyable> ("DynamixelIO", init<std::string, int> ())
        .def("get_cached_parameters", &DynamixelIOWrap::getCachedParameters, return_value_policy<manage_new_object>())
        .def ("ping", &DynamixelIOWrap::ping,
              "Returns True if specified servo_id is connected and responding properly, False otherwise.\n\n"
              "Usage:\n"
              ">>> dxl_io.ping(4)\n"
              "True\n"
              ">>> dxl_io.ping(20)\n"
              "False")
        .def ("reset_overload_error", &DynamixelIOWrap::resetOverloadError)
        
        // ****************************** GETTERS ******************************** //
        .def ("get_model_number", &DynamixelIOWrap::getModelNumber)
        .def ("get_firmware_version", &DynamixelIOWrap::getFirmwareVersion)
        .def ("get_baud_rate", &DynamixelIOWrap::getBaudRate)
        .def ("get_return_delay_time", &DynamixelIOWrap::getReturnDelayTime)
        
        .def ("get_angle_limits", &DynamixelIOWrap::getAngleLimits)
        .def ("get_cw_angle_limit", &DynamixelIOWrap::getCWAngleLimit)
        .def ("get_ccw_angle_limit", &DynamixelIOWrap::getCCWAngleLimit)
        
        .def ("get_voltage_limits", &DynamixelIOWrap::getVoltageLimits)
        .def ("get_min_voltage_limit", &DynamixelIOWrap::getMinVoltageLimit)
        .def ("get_max_voltage_limit", &DynamixelIOWrap::getMaxVoltageLimit)
        
        .def ("get_temperature_limit", &DynamixelIOWrap::getTemperatureLimit)
        .def ("get_max_torque", &DynamixelIOWrap::getMaxTorque)
        .def ("get_alarm_led", &DynamixelIOWrap::getAlarmLed)
        .def ("get_alarm_shutdown", &DynamixelIOWrap::getAlarmShutdown)
        .def ("get_torque_enable", &DynamixelIOWrap::getTorqueEnable)
        .def ("get_led_status", &DynamixelIOWrap::getLedStatus)
        
        .def ("get_compliance_margins", &DynamixelIOWrap::getComplianceMargins)
        .def ("get_cw_compliance_margin", &DynamixelIOWrap::getCWComplianceMargin)
        .def ("get_ccw_compliance_margin", &DynamixelIOWrap::getCCWComplianceMargin)
        
        .def ("get_compliance_slopes", &DynamixelIOWrap::getComplianceSlopes)
        .def ("get_cw_compliance_slope", &DynamixelIOWrap::getCWComplianceSlope)
        .def ("get_ccw_compliance_slope", &DynamixelIOWrap::getCCWComplianceSlope)
        
        .def ("get_target_position", &DynamixelIOWrap::getTargetPosition)
        .def ("get_target_velocity", &DynamixelIOWrap::getTargetVelocity)
        .def ("get_torque_limit", &DynamixelIOWrap::getTorqueLimit)
        
        .def ("get_position", &DynamixelIOWrap::getPosition)
        .def ("get_velocity", &DynamixelIOWrap::getVelocity)
        .def ("get_load", &DynamixelIOWrap::getLoad)
        .def ("get_voltage", &DynamixelIOWrap::getVoltage)
        .def ("get_temperature", &DynamixelIOWrap::getTemperature)
        .def ("get_moving", &DynamixelIOWrap::getMoving)
        
        .def ("get_feedback", &DynamixelIOWrap::getFeedback)

        // ****************************** SETTERS ******************************** //
        .def ("set_id", &DynamixelIOWrap::setId)
        .def ("set_baud_rate", &DynamixelIOWrap::setBaudRate)
        .def ("set_return_delay_time", &DynamixelIOWrap::setReturnDelayTime)
        
        .def ("set_angle_limits", &DynamixelIOWrap::setAngleLimits)
        .def ("set_cw_angle_limit", &DynamixelIOWrap::setCWAngleLimit)
        .def ("set_ccw_angle_limit", &DynamixelIOWrap::setCCWAngleLimit)
    
        .def ("set_voltage_limits", &DynamixelIOWrap::setVoltageLimits)
        .def ("set_min_voltage_limit", &DynamixelIOWrap::setMinVoltageLimit)
        .def ("set_max_voltage_limit", &DynamixelIOWrap::setMaxVoltageLimit)
    
        .def ("set_temperature_limit", &DynamixelIOWrap::setTemperatureLimit)
        .def ("set_max_torque", &DynamixelIOWrap::setMaxTorque)
        .def ("set_alarm_led", &DynamixelIOWrap::setAlarmLed)
        .def ("set_alarm_shutdown", &DynamixelIOWrap::setAlarmShutdown)
        .def ("set_torque_enable", &DynamixelIOWrap::setTorqueEnable)
        .def ("set_led", &DynamixelIOWrap::setLed)

        .def ("set_compliance_margins", &DynamixelIOWrap::setComplianceMargins)
        .def ("set_cw_compliance_margin", &DynamixelIOWrap::setCWComplianceMargin)
        .def ("set_ccw_compliance_margin", &DynamixelIOWrap::setCCWComplianceMargin)
        
        .def ("set_compliance_slopes", &DynamixelIOWrap::setComplianceSlopes)
        .def ("set_cw_compliance_slope", &DynamixelIOWrap::setCWComplianceSlope)
        .def ("set_ccw_compliance_slope", &DynamixelIOWrap::setCCWComplianceSlope)

        .def ("set_position", &DynamixelIOWrap::setPosition)
        .def ("set_velocity", &DynamixelIOWrap::setVelocity)
        .def ("set_torque_limit", &DynamixelIOWrap::setTorqueLimit)
        
        // ************************* SYNC_WRITE METHODS *************************** //
        .def ("set_multi_position", &DynamixelIOWrap::setMultiPosition)
        .def ("set_multi_velocity", &DynamixelIOWrap::setMultiVelocity)
        .def ("set_multi_position_velocity", &DynamixelIOWrap::setMultiPositionVelocity)
        .def ("set_multi_compliance_margins", &DynamixelIOWrap::setMultiComplianceMargins)
        .def ("set_multi_compliance_slopes", &DynamixelIOWrap::setMultiComplianceSlopes)
        .def ("set_multi_torque_enabled", &DynamixelIOWrap::setMultiTorqueEnabled)
        .def ("set_multi_torque_limit", &DynamixelIOWrap::setMultiTorqueLimit)

        .def ("read", &DynamixelIOWrap::read)
        .def ("write", &DynamixelIOWrap::write)
        .def ("sync_write", &DynamixelIOWrap::syncWrite)
        ;
}
