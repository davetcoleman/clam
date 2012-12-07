/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics 
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include "sickdefines.h"
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

namespace gbxsickacfr {

std::string cmdToString( uChar command )
{
    switch( command )
    {
    case ACK:
        return "ACK(0x06)";
    case NACK:
        return "NACK(0x15)";

    case CMD_INIT_AND_RESET:
        return "INIT_AND_RESET(0x10)";
    case CMD_SWITCH_OPERATING_MODE:
        return "SWITCH_OPERATING_MODE(0x20)";
    case CMD_REQUEST_MEASURED_VALUES:
        return "REQUEST_MEASURED_VALUES(0x30)";
    case CMD_REQUEST_LMS_STATUS:
        return "REQUEST_LMS_STATUS(0x31)";
    case CMD_REQUEST_ERROR_OR_TEST_MESSAGE:
        return "REQUEST_ERROR_OR_TEST_MESSAGE(0x32)";
    case CMD_REQUEST_OPERATING_DATA_COUNTER:
        return "REQUEST_OPERATING_DATA_COUNTER(0x35)";
    case CMD_REQUEST_MEAN_MEASURED_VALUES:
        return "REQUEST_MEAN_MEASURED_VALUES(0x36)";
    case CMD_REQUEST_MEASURED_VALUE_SUBRANGE:
        return "REQUEST_MEASURED_VALUE_SUBRANGE(0x37)";
    case CMD_REQUEST_LMS_TYPE:
        return "REQUEST_LMS_TYPE(0x3A)";
    case CMD_SWITCH_VARIANT:
        return "SWITCH_VARIANT(0x3B)";
    case CMD_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES:
        return "REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES(0x3E)";
    case CMD_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE:
        return "REQUEST_MEAN_MEASURED_VALUE_SUBRANGE(0x3F)";
    case CMD_CONFIGURE_FIELDS:
        return "CONFIGURE_FIELDS(0x40)";
    case CMD_SWITCH_ACTIVE_FIELD_SET:
        return "SWITCH_ACTIVE_FIELD_SET(0x41)";
    case CMD_CHANGE_PASSWORD:
        return "CHANGE_PASSWORD(0x42)";
    case CMD_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE:
        return "REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE(0x44)";
    case CMD_REQUEST_CONFIGURED_FIELDS:
        return "REQUEST_CONFIGURED_FIELDS(0x45)";
    case CMD_START_TEACH_MODE_FOR_FIELD_CONFIGURATION:
        return "START_TEACH_MODE_FOR_FIELD_CONFIGURATION(0x46)";
    case CMD_REQUEST_STATUS_OF_FIELD_OUTPUTS:
        return "REQUEST_STATUS_OF_FIELD_OUTPUTS(0x4A)";
    case CMD_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE:
        return "DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE(0x66)";
    case CMD_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID:
        return "DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID(0x69)";
    case CMD_REQUEST_LMS_CONFIGURATION:
        return "REQUEST_LMS_CONFIGURATION(0x74)";
    case CMD_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA:
        return "REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA(0x75)";
    case CMD_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES:
        return "REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES(0x76)";
    case CMD_CONFIGURE_LMS:
        return "CONFIGURE_LMS(0x77)";
    case CMD_CONFIGURE_LMS_CONTINUED:
        return "CONFIGURE_LMS_CONTINUED(0x7C)";

    case ACK_INIT_AND_RESET:
        return "ACK_INIT_AND_RESET";
    case ACK_SWITCH_OPERATING_MODE:
        return "ACK_SWITCH_OPERATING_MODE";
    case ACK_REQUEST_MEASURED_VALUES:
        return "ACK_REQUEST_MEASURED_VALUES";
    case ACK_REQUEST_LMS_STATUS:
        return "ACK_REQUEST_LMS_STATUS";
    case ACK_REQUEST_ERROR_OR_TEST_MESSAGE:
        return "ACK_REQUEST_ERROR_OR_TEST_MESSAGE";
    case ACK_REQUEST_OPERATING_DATA_COUNTER:
        return "ACK_REQUEST_OPERATING_DATA_COUNTER";
    case ACK_REQUEST_MEAN_MEASURED_VALUES:
        return "ACK_REQUEST_MEAN_MEASURED_VALUES";
    case ACK_REQUEST_MEASURED_VALUE_SUBRANGE:
        return "ACK_REQUEST_MEASURED_VALUE_SUBRANGE";
    case ACK_REQUEST_LMS_TYPE:
        return "ACK_REQUEST_LMS_TYPE";
    case ACK_SWITCH_VARIANT:
        return "ACK_SWITCH_VARIANT";
    case ACK_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES:
        return "ACK_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES";
    case ACK_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE:
        return "ACK_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE";
    case ACK_CONFIGURE_FIELDS:
        return "ACK_CONFIGURE_FIELDS";
    case ACK_SWITCH_ACTIVE_FIELD_SET:
        return "ACK_SWITCH_ACTIVE_FIELD_SET";
    case ACK_CHANGE_PASSWORD:
        return "ACK_CHANGE_PASSWORD";
    case ACK_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE:
        return "ACK_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE";
    case ACK_REQUEST_CONFIGURED_FIELDS:
        return "ACK_REQUEST_CONFIGURED_FIELDS";
    case ACK_START_TEACH_MODE_FOR_FIELD_CONFIGURATION:
        return "ACK_START_TEACH_MODE_FOR_FIELD_CONFIGURATION";
    case ACK_REQUEST_STATUS_OF_FIELD_OUTPUTS:
        return "ACK_REQUEST_STATUS_OF_FIELD_OUTPUTS";
    case ACK_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE:
        return "ACK_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE";
    case ACK_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID:
        return "ACK_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID";
    case ACK_REQUEST_LMS_CONFIGURATION:
        return "ACK_REQUEST_LMS_CONFIGURATION";
    case ACK_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA:
        return "ACK_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA";
    case ACK_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES:
        return "ACK_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES";
    case ACK_CONFIGURE_LMS:
        return "ACK_CONFIGURE_LMS";
    case ACK_CONFIGURE_LMS_CONTINUED:
        return "ACK_CONFIGURE_LMS_CONTINUED";

    case RESP_SOFTWARE_RESET_CONFIRM:
        return "RESP_SOFTWARE_RESET_CONFIRM";

    case RESP_INCORRECT_COMMAND:
        return "RESP_INCORRECT_COMMAND";

    default:
        std::stringstream ss;
        ss << " ?? Unknown ?? (0x"<<hex<<std::setfill('0')<<std::setw(2)<<(int)(command)<<")";
        return ss.str();
    }            
}

std::string operatingModeToString( uChar operatingMode )
{
    switch ( operatingMode )
    {
    case OPERATING_MODE_INSTALLATION:
        return "OPERATING_MODE_INSTALLATION(0x00)";
    case OPERATING_MODE_DIAGNOSIS:
        return "OPERATING_MODE_DIAGNOSIS(0x10)";
    case OPERATING_MODE_MIN_MEASURED_CONTINUOUS:
        return "OPERATING_MODE_MIN_MEASURED_CONTINUOUS(0x20)";
    case OPERATING_MODE_MIN_MEASURED_IN_FIELD:
        return "OPERATING_MODE_MIN_MEASURED_IN_FIELD(0x21)";
    case OPERATING_MODE_MIN_VERTICAL_DISTANCE_CONTINUOUS:
        return "OPERATING_MODE_MIN_VERTICAL_DISTANCE_CONTINUOUS(0x22)";
    case OPERATING_MODE_MIN_VERTICAL_DISTANCE_IN_FIELD:
        return "OPERATING_MODE_MIN_VERTICAL_DISTANCE_IN_FIELD(0x23)";
    case OPERATING_MODE_ALL_MEASURED_CONTINUOUS:
        return "OPERATING_MODE_ALL_MEASURED_CONTINUOUS(0x24)";
    case OPERATING_MODE_MEASURED_ON_REQUEST:
        return "OPERATING_MODE_MEASURED_ON_REQUEST(0x25)"; // (this is the default setting)
    case OPERATING_MODE_MEAN_MEASURED_CONTINUOUS:
        return "OPERATING_MODE_MEAN_MEASURED_CONTINUOUS(0x26)";
    case OPERATING_MODE_MEASURED_SUBRANGE_CONTINUOUS:
        return "OPERATING_MODE_MEASURED_SUBRANGE_CONTINUOUS(0x27)";
    case OPERATING_MODE_MEAN_MEASURED_SUBRANGE_CONTINUOUS:
        return "OPERATING_MODE_MEAN_MEASURED_SUBRANGE_CONTINUOUS(0x28)";
    case OPERATING_MODE_MEASURED_WITH_FIELD_VALUES_CONTINUOUS:
        return "OPERATING_MODE_MEASURED_WITH_FIELD_VALUES_CONTINUOUS(0x29)";
    case OPERATING_MODE_MEASURED_PARTIAL_CONTINUOUS:
        return "OPERATING_MODE_MEASURED_PARTIAL_CONTINUOUS(0x2A)";
    case OPERATING_MODE_MEASURED_N_PARTIAL_CONTINUOUS:
        return "OPERATING_MODE_MEASURED_N_PARTIAL_CONTINUOUS(0x2B)";
    case OPERATING_MODE_MEASURED_PER_SEGMENT_CONTINUOUS:
        return "OPERATING_MODE_MEASURED_PER_SEGMENT_CONTINUOUS(0x2C)";
    case OPERATING_MODE_NAV_DATA_RECORDS:
        return "OPERATING_MODE_NAV_DATA_RECORDS(0x2E)";
    case OPERATING_MODE_ALL_MEASURED_PLUS_REFLECTIVITY_SUBRANGE_CONTINUOUS:
        return "OPERATING_MODE_ALL_MEASURED_PLUS_REFLECTIVITY_SUBRANGE_CONTINUOUS(0x50)";
    case OPERATING_MODE_TEST_PASSWORD:
        return "OPERATING_MODE_TEST_PASSWORD(0x30)";
    case OPERATING_MODE_SET_BAUDRATE_38400 :
        return "OPERATING_MODE_SET_BAUDRATE_38400 (0x40)";
    case OPERATING_MODE_SET_BAUDRATE_19200 :
        return "OPERATING_MODE_SET_BAUDRATE_19200 (0x41)";
    case OPERATING_MODE_SET_BAUDRATE_9600  :
        return "OPERATING_MODE_SET_BAUDRATE_9600  (0x42)";
    case OPERATING_MODE_SET_BAUDRATE_500000:
        return "OPERATING_MODE_SET_BAUDRATE_500000(0x48)";        
    default:
        stringstream ss;
        ss << " ?? Unknown operating mode ?? (0x"<<hex<<(int)(operatingMode)<<dec<<")";
        return ss.str();
    }
}

std::string measuringModeToString( uChar mode )
{
    switch ( mode )
    {
    case MEASURING_MODE_8m80m_FIELD_AB_DAZZLE:
        return "MEASURING_MODE_8m80m_FIELD_AB_DAZZLE(0x00)";
    case MEASURING_MODE_8m80m_REFLECTOR8LEVELS:
        return "MEASURING_MODE_8m80m_REFLECTOR8LEVELS(0x01)";
    case MEASURING_MODE_8m80m_FIELD_ABC:
        return "MEASURING_MODE_8m80m_FIELD_ABC(0x02)";
    case MEASURING_MODE_16m_REFLECTOR4LEVELS:
        return "MEASURING_MODE_16m_REFLECTOR4LEVELS(0x03)";
    case MEASURING_MODE_16m_FIELD_AB:
        return "MEASURING_MODE_16m_FIELD_AB(0x04)";
    case MEASURING_MODE_32m_FIELD_AB:
        return "MEASURING_MODE_32m_FIELD_AB(0x05)";
    case MEASURING_MODE_32m_FIELD_A:
        return "MEASURING_MODE_32_FIELD_A(0x06)";
    default:
        stringstream ss;
        ss << " ?? Unknown measuring mode ?? (0x"<<hex<<(int)(mode)<<dec<<")";
        return ss.str();
    }
}

std::string statusToString( uChar status )
{
    stringstream ss;
    switch ( status & STATUS_GENERAL_MASK )
    {
    case STATUS_OK:
        ss << "STATUS_OK";
        break;
    case STATUS_INFO:
        ss << "STATUS_INFO";
        break;
    case STATUS_WARNING:
        ss << "STATUS_WARNING";
        break;
    case STATUS_ERROR:
        ss << "STATUS_ERROR";
        break;
    case STATUS_FATAL_ERROR:
        ss << "STATUS_FATAL_ERROR";
        break;
    default:
        ss << " ?? Unknown general status ?? : " << (status&STATUS_GENERAL_MASK);
        break;
    }

    if ( status & STATUS_RESTART_MASK )
        ss << ", RESTART";
    if ( status & STATUS_IMPLAUSIBLE_MEASURED_VALUES_MASK )
        ss << ", IMPLAUSIBLE_MEASURED_VALUES";
    if ( status & STATUS_POLLUTION_MASK )
        ss << ", POLLUTION";

    return ss.str();
}

bool generalStatusIsWarn( uChar generalStatus )
{
// AlexB: don't warn on 'INFO', just warn on 'WARNING'.
//     return ( generalStatus == STATUS_INFO ||
//              generalStatus == STATUS_WARNING );
    return ( generalStatus == STATUS_WARNING );
}

bool generalStatusIsError( uChar generalStatus )
{
    return ( generalStatus == STATUS_ERROR ||
             generalStatus == STATUS_FATAL_ERROR );
}

int baudRateToInt( uint16_t baudRate )
{
    switch ( baudRate )
    {
    case 0x8001:
        return 500000;
    case 0x8019:
        return 38400;
    case 0x8033:
        return 19200;
    case 0x8067:
        return 9600;
    default:
        cout<<"TRACE(sickdefines.cpp): Unknown baud rate: " << hex << baudRate << dec << endl;
        return -1;
    }
}

uChar baudRateIntToOperatingMode( int baudRate )
{
    switch ( baudRate )
    {
    case 9600:
        return OPERATING_MODE_SET_BAUDRATE_9600;
    case 19200:
        return OPERATING_MODE_SET_BAUDRATE_19200;
    case 38400:
        return OPERATING_MODE_SET_BAUDRATE_38400;
    case 500000:
        return OPERATING_MODE_SET_BAUDRATE_500000;
    default:
        cout<<"TRACE(sickdefines.cpp): baudRateIntToOperatingMode: Unknown baudrate: " << baudRate << endl;
        return 0xff;
    }
}

std::string measuredValueUnitToString( uChar unit )
{
    switch ( unit )
    {
    case MEASURED_VALUE_UNIT_CM:
        return "MEASURED_VALUE_UNIT_CM";
    case MEASURED_VALUE_UNIT_MM:
        return "MEASURED_VALUE_UNIT_MM";
    default:
        stringstream ss;
        ss << "?? Unknown measuredValueUnit ?? (0x"<<hex<<unit<<")";
        return ss.str();
    }        
}

std::string permanentBaudRateToString( uChar permanentBaudRate )
{
    switch ( permanentBaudRate )
    {
    case 0x00:
        return "Reset to 9600 on power-cycle";
    case 0x01:
        return "Retain baud rate through power-cycle";
    default:
        stringstream ss;
        ss << "?? Unknown permanentBaudRate ?? (0x"<<hex<<permanentBaudRate<<")";
        return ss.str();
    }
}

std::string modeSwitchSuccessToString( uChar success )
{
    switch ( success )
    {
    case MODE_SWITCH_SUCCESS_OK:
        return "MODE_SWITCH_SUCCESS_OK(0x00)";
    case MODE_SWITCH_FAIL_BAD_PASSWORD:
        return "MODE_SWITCH_FAIL_BAD_PASSWORD(0x01)";
    case MODE_SWITCH_FAIL_LMS_FAULT:
        return "MODE_SWITCH_FAIL_LMS_FAULT(0x02)";
    default:
        stringstream ss;
        ss << " ?? Unknown mode switch success ?? (0x"<<hex<<(int)(success)<<dec<<")";
        return ss.str();
    }
}

std::string sensitivityToString( uChar sens )
{
    switch ( sens )
    {
    case SENSITIVITY_STANDARD:
        return "SENSITIVITY_STANDARD(approx 30m at 10% reflectivity)";
    case SENSITIVITY_MEDIUM:
        return "SENSITIVITY_MEDIUM(approx 25m at 10% reflectivity)";
    case SENSITIVITY_LOW:
        return "SENSITIVITY_LOW(approx 20m at 10% reflectivity)";
    case SENSITIVITY_HIGH:
        return "SENSITIVITY_HIGH(approx 42m at 10% reflectivity)";
    default:
        stringstream ss;
        ss << " ?? Unknown sensitivity ?? (0x"<<hex<<(int)(sens)<<dec<<")";
        return ss.str();
    }
}

std::string configurationSuccessToString( uChar success )
{
    switch ( success )
    {
    case CONFIGURATION_SUCCESS:
        return "CONFIGURATION_SUCCESS";
    case CONFIGURATION_FAIL:
        return "CONFIGURATION_FAIL";
    default:
        stringstream ss;
        ss << " ?? Unknown configuration success ?? (0x"<<hex<<(int)(success)<<dec<<")";
        return ss.str();
    }
}

std::string switchVariantSuccessToString( uChar success )
{
    switch ( success )
    {
    case SWITCH_VARIANT_SUCCESS:
        return "SWITCH_VARIANT_SUCCESS";
    case SWITCH_VARIANT_FAIL:
        return "SWITCH_VARIANT_FAIL";
    default:
        stringstream ss;
        ss << " ?? Unknown switch-variant success ?? (0x"<<hex<<(int)(success)<<dec<<")";
        return ss.str();
    }
}

std::string errorTypeToString( uChar errorType )
{
    // Bottom two bits are the same as status
    stringstream ss;
    ss << statusToString( errorType && 0x03 );

    if ( errorType & ERROR_TYPE_NO_LONGER_RELEVANT_MASK )
    {
        ss << " (no longer relevant)";
    }
    else
    {
        ss << " (still relevant)";
    }
    return ss.str();
}

bool errorTypeIsWarn( uChar errorType )
{
    // Bottom two bits are the same as status
    return generalStatusIsWarn( errorType && 0x03 );
}

bool errorTypeIsError( uChar errorType )
{
    // Bottom two bits are the same as status
    return generalStatusIsError( errorType && 0x03 );
}

std::string errorCodeToString( uChar errorCode )
{
    switch( errorCode )
    {
    case ERROR_CODE_PEAK_COMPARATOR_TEST:
        return "ERROR_CODE_PEAK_COMPARATOR_TEST(06)";
    case ERROR_CODE_STOP_COMPARATOR_TEST:
        return "ERROR_CODE_STOP_COMPARATOR_TEST(07)";
    case ERROR_CODE_TDC_INIT_AND_GATE_FUNCTION_TEST:
        return "ERROR_CODE_TDC_INIT_AND_GATE_FUNCTION_TEST(15)";
    case ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_1:
        return "ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_1(17)";
    case ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_1:
        return "ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_1(18)";
    case ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_2:
        return "ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_2(19)";
    case ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_2:
        return "ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_2(20)";
    case ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_0:
        return "ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_0(21)";
    case ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_1:
        return "ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_1(22)";
    case ERROR_CODE_OUTPUT_A_DEFECTIVE:
        return "ERROR_CODE_OUTPUT_A_DEFECTIVE(27)";
    case ERROR_CODE_OUTPUT_B_DEFECTIVE:
        return "ERROR_CODE_OUTPUT_B_DEFECTIVE(28)";
    case ERROR_CODE_NUM_MOTOR_REVOLUTIONS:
        return "ERROR_CODE_NUM_MOTOR_REVOLUTIONS(29)";
    case ERROR_CODE_CALIBRATION_POLLUTION:
        return "ERROR_CODE_CALIBRATION_POLLUTION(37)";
    case ERROR_CODE_TIMEOUT_ON_TDC_CALIBRATION:
        return "ERROR_CODE_TIMEOUT_ON_TDC_CALIBRATION(39)";
    case ERROR_CODE_1_MEASUREMENT_VALUE_MISSING:
        return "ERROR_CODE_1_MEASUREMENT_VALUE_MISSING(45)";
    case ERROR_CODE_1_SCAN_MISSING_SCAN_LASTS_TOO_LONG:
        return "ERROR_CODE_1_SCAN_MISSING_SCAN_LASTS_TOO_LONG(46)";
    case ERROR_CODE_REFERENCE_TARGET_LOAD_PULSE_WIDTH_IMPLAUSIBLE:
        return "ERROR_CODE_REFERENCE_TARGET_LOAD_PULSE_WIDTH_IMPLAUSIBLE(47)";
    case ERROR_CODE_CALIBRATION_OF_LASER_POWER:
        return "ERROR_CODE_CALIBRATION_OF_LASER_POWER(48)";
    case ERROR_CODE_LASER_POWER_OUTSIDE_50_PCT_140_PCT:
        return "ERROR_CODE_LASER_POWER_OUTSIDE_50_PCT_140_PCT(49)";
    case ERROR_CODE_INITIALISATION_TDC_MO_CHANNEL_0_AND_1:
        return "ERROR_CODE_INITIALISATION_TDC_MO_CHANNEL_0_AND_1(50)";
    case ERROR_CODE_DA_AD_TEST_STOP_BRANCH:
        return "ERROR_CODE_DA_AD_TEST_STOP_BRANCH(51)";
    case ERROR_CODE_DA_AD_TEST_PEAK_BRANCH:
        return "ERROR_CODE_DA_AD_TEST_PEAK_BRANCH(52)";
    case ERROR_CODE_FLASH_WRITTEN:
        return "ERROR_CODE_FLASH_WRITTEN(53)";
    case ERROR_CODE_POLLUTION_CHANNEL_MEASUREMENT_WITHOUT_ACTIVE_TRANSMITTER:
        return "ERROR_CODE_POLLUTION_CHANNEL_MEASUREMENT_WITHOUT_ACTIVE_TRANSMITTER(54)";
    case ERROR_CODE_NO_TWO_DIFFERENT_ANGLES_DETECTED_ON_LASER_POWER_CALIBRATION:
        return "ERROR_CODE_NO_TWO_DIFFERENT_ANGLES_DETECTED_ON_LASER_POWER_CALIBRATION(55)";
    case ERROR_CODE_HARDWARE_WATCHDOG_DEFECTIVE:
        return "ERROR_CODE_HARDWARE_WATCHDOG_DEFECTIVE(56)";
    case ERROR_CODE_NO_ZERO_INDEX_SIGNAL_AVAILABLE:
        return "ERROR_CODE_NO_ZERO_INDEX_SIGNAL_AVAILABLE(57)";
    case ERROR_CODE_SLAVE_CANNOT_SYNC_DURING_INIT:
        return "ERROR_CODE_SLAVE_CANNOT_SYNC_DURING_INIT(58)";
    case ERROR_CODE_SYNC_LOST_IN_OPERATING_STATE:
        return "ERROR_CODE_SYNC_LOST_IN_OPERATING_STATE(59)";
    case ERROR_CODE_SYNC_CYCLE_FROM_MASTER_MISSING:
        return "ERROR_CODE_SYNC_CYCLE_FROM_MASTER_MISSING(60)";
    case ERROR_CODE_HARDWARE_UNSUITABLE_FOR_SYNC_IN_SLAVE_OPERATING_MODE:
        return "ERROR_CODE_HARDWARE_UNSUITABLE_FOR_SYNC_IN_SLAVE_OPERATING_MODE(61)";
    case ERROR_CODE_WRONG_DIP_SWITCH_POSITION:
        return "ERROR_CODE_WRONG_DIP_SWITCH_POSITION(62)";
    case ERROR_CODE_REF_TARGET_SMALLEST_PULSE_WIDTH_TOO_SMALL:
        return "ERROR_CODE_REF_TARGET_SMALLEST_PULSE_WIDTH_TOO_SMALL(86)";
    case ERROR_CODE_REF_TARGET_LARGEST_PULSE_WIDTH_TOO_LARGE:
        return "ERROR_CODE_REF_TARGET_LARGEST_PULSE_WIDTH_TOO_LARGE(87)";
    case ERROR_CODE_REF_TARGET_PULSE_WIDTH_SPECTRUM_TOO_LARGE:
        return "ERROR_CODE_REF_TARGET_PULSE_WIDTH_SPECTRUM_TOO_LARGE(88)";
    case ERROR_CODE_REF_TARGET_REF_TABLE_LESS_THAN_2_CYCLES_UPDATE:
        return "ERROR_CODE_REF_TARGET_REF_TABLE_LESS_THAN_2_CYCLES_UPDATE(89)";
    case ERROR_CODE_REF_TARGET_REFLECTIVITY_MEASUREMENT_CANNOT_BE_CALIBRATED:
        return "ERROR_CODE_REF_TARGET_REFLECTIVITY_MEASUREMENT_CANNOT_BE_CALIBRATED(91)";
    case ERROR_CODE_REF_TARGET_TEACH_IN_MODE_IS_NOT_COMPLETED:
        return "ERROR_CODE_REF_TARGET_TEACH_IN_MODE_IS_NOT_COMPLETED(92)";
    case ERROR_CODE_OUT_OF_MEMORY_IN_MEASUREMENT_ROUTINE:
        return "ERROR_CODE_OUT_OF_MEMORY_IN_MEASUREMENT_ROUTINE(124)";
    case ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ROUTINE:
        return "ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ROUTINE(125)";
    case ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ANGULAR_TABLE:
        return "ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ANGULAR_TABLE(126)";
    default:
        stringstream ss;
        ss << " ?? Unknown error code: " <<dec<< errorCode << " ?? ";
        return ss.str();
    }
}

}
