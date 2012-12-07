/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics 
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef SICK_ACFR_DRIVER_SICKDEFINES_H
#define SICK_ACFR_DRIVER_SICKDEFINES_H

#include <string>
#include <stdint.h>

namespace gbxsickacfr {

    typedef unsigned char uChar;

    //////////////////////////////////////////////////////////////////////
    //
    // Random SICK crap
    //
    const uChar STX  = 0x02;     // Start of Text
    const uChar ACK  = 0x06;
    const uChar NACK = 0x15;
    const uChar ADDRESS = 0x00;

    const int MAX_SICK_TELEGRAM_LENGTH = 812; // bytes

    const int MAX_RESPONSE_TIME_MS = 60;

    //////////////////////////////////////////////////////////////////////
    //
    // Command Types
    //
    const uChar CMD_INIT_AND_RESET = 0x10;
    const uChar CMD_SWITCH_OPERATING_MODE = 0x20;
    const uChar CMD_REQUEST_MEASURED_VALUES = 0x30;
    const uChar CMD_REQUEST_LMS_STATUS = 0x31;
    const uChar CMD_REQUEST_ERROR_OR_TEST_MESSAGE = 0x32;
    const uChar CMD_REQUEST_OPERATING_DATA_COUNTER = 0x35;
    const uChar CMD_REQUEST_MEAN_MEASURED_VALUES = 0x36;
    const uChar CMD_REQUEST_MEASURED_VALUE_SUBRANGE = 0x37;
    const uChar CMD_REQUEST_LMS_TYPE = 0x3A;
    const uChar CMD_SWITCH_VARIANT = 0x3B;
    const uChar CMD_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES = 0x3E;
    const uChar CMD_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE = 0x3F;
    const uChar CMD_CONFIGURE_FIELDS = 0x40;
    const uChar CMD_SWITCH_ACTIVE_FIELD_SET = 0x41;
    const uChar CMD_CHANGE_PASSWORD = 0x42;
    const uChar CMD_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE = 0x44;
    const uChar CMD_REQUEST_CONFIGURED_FIELDS = 0x45;
    const uChar CMD_START_TEACH_MODE_FOR_FIELD_CONFIGURATION = 0x46;
    const uChar CMD_REQUEST_STATUS_OF_FIELD_OUTPUTS = 0x4A;
    const uChar CMD_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE = 0x66;
    const uChar CMD_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID = 0x69;
    const uChar CMD_REQUEST_LMS_CONFIGURATION = 0x74;
    const uChar CMD_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA = 0x75;
    const uChar CMD_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES = 0x76;
    const uChar CMD_CONFIGURE_LMS = 0x77;
    const uChar CMD_CONFIGURE_LMS_CONTINUED = 0x7C;
    //////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////
    //
    // Acknowledgement types
    //
    inline uChar ack( uChar command ) { return (uChar)(command + 0x80); }

    // Need to do this so they can be used in constant expressions (eg in switch stmts)
    const uChar ACK_INIT_AND_RESET = CMD_INIT_AND_RESET + 0x80;
    const uChar ACK_SWITCH_OPERATING_MODE = CMD_SWITCH_OPERATING_MODE + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUES = CMD_REQUEST_MEASURED_VALUES + 0x80;
    const uChar ACK_REQUEST_LMS_STATUS = CMD_REQUEST_LMS_STATUS + 0x80;
    const uChar ACK_REQUEST_ERROR_OR_TEST_MESSAGE = CMD_REQUEST_ERROR_OR_TEST_MESSAGE + 0x80;
    const uChar ACK_REQUEST_OPERATING_DATA_COUNTER = CMD_REQUEST_OPERATING_DATA_COUNTER + 0x80;
    const uChar ACK_REQUEST_MEAN_MEASURED_VALUES = CMD_REQUEST_MEAN_MEASURED_VALUES + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUE_SUBRANGE = CMD_REQUEST_MEASURED_VALUE_SUBRANGE + 0x80;
    const uChar ACK_REQUEST_LMS_TYPE = CMD_REQUEST_LMS_TYPE + 0x80;
    const uChar ACK_SWITCH_VARIANT = CMD_SWITCH_VARIANT + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES = 
                CMD_REQUEST_MEASURED_VALUE_WITH_FIELD_VALUES + 0x80;
    const uChar ACK_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE = 
                CMD_REQUEST_MEAN_MEASURED_VALUE_SUBRANGE + 0x80;
    const uChar ACK_CONFIGURE_FIELDS = CMD_CONFIGURE_FIELDS + 0x80;
    const uChar ACK_SWITCH_ACTIVE_FIELD_SET = CMD_SWITCH_ACTIVE_FIELD_SET + 0x80;
    const uChar ACK_CHANGE_PASSWORD = CMD_CHANGE_PASSWORD + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE = 
                CMD_REQUEST_MEASURED_VALUES_AND_REFLECTIVITY_VALUE_SUBRANGE + 0x80;
    const uChar ACK_REQUEST_CONFIGURED_FIELDS = CMD_REQUEST_CONFIGURED_FIELDS + 0x80;
    const uChar ACK_START_TEACH_MODE_FOR_FIELD_CONFIGURATION = 
                CMD_START_TEACH_MODE_FOR_FIELD_CONFIGURATION + 0x80;
    const uChar ACK_REQUEST_STATUS_OF_FIELD_OUTPUTS = CMD_REQUEST_STATUS_OF_FIELD_OUTPUTS + 0x80;
    const uChar ACK_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE = 
                CMD_DEFINE_PERMANENT_BAUD_RATE_OR_LMS_TYPE + 0x80;
    const uChar ACK_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID =
                CMD_DEFINE_ANGULAR_RANGE_FOR_POSITIONING_AID + 0x80;
    const uChar ACK_REQUEST_LMS_CONFIGURATION = CMD_REQUEST_LMS_CONFIGURATION + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA =
                CMD_REQUEST_MEASURED_VALUE_WITH_REFLECTIVITY_DATA + 0x80;
    const uChar ACK_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES = 
                CMD_REQUEST_MEASURED_VALUES_IN_CARTESIAN_COORDINATES + 0x80;
    const uChar ACK_CONFIGURE_LMS = CMD_CONFIGURE_LMS + 0x80;
    const uChar ACK_CONFIGURE_LMS_CONTINUED = CMD_CONFIGURE_LMS_CONTINUED + 0x80;
    
    // Response Types (in addition to acknowledgement types)
    const uChar RESP_SOFTWARE_RESET_CONFIRM = 0x91;
    const uChar RESP_INCORRECT_COMMAND = 0x92;

    std::string cmdToString( uChar command );
    

    //////////////////////////////////////////////////////////////////////
    //
    // Operating Modes
    //
    const uChar OPERATING_MODE_INSTALLATION = 0x00;

    const uChar OPERATING_MODE_DIAGNOSIS = 0x10;

    const uChar OPERATING_MODE_MIN_MEASURED_CONTINUOUS = 0x20;
    const uChar OPERATING_MODE_MIN_MEASURED_IN_FIELD = 0x21;
    const uChar OPERATING_MODE_MIN_VERTICAL_DISTANCE_CONTINUOUS = 0x22;
    const uChar OPERATING_MODE_MIN_VERTICAL_DISTANCE_IN_FIELD = 0x23;
    const uChar OPERATING_MODE_ALL_MEASURED_CONTINUOUS = 0x24;
    const uChar OPERATING_MODE_MEASURED_ON_REQUEST = 0x25; // (this is the default setting)
    const uChar OPERATING_MODE_MEAN_MEASURED_CONTINUOUS = 0x26;
    const uChar OPERATING_MODE_MEASURED_SUBRANGE_CONTINUOUS = 0x27;
    const uChar OPERATING_MODE_MEAN_MEASURED_SUBRANGE_CONTINUOUS = 0x28;
    const uChar OPERATING_MODE_MEASURED_WITH_FIELD_VALUES_CONTINUOUS = 0x29;
    const uChar OPERATING_MODE_MEASURED_PARTIAL_CONTINUOUS = 0x2A;
    const uChar OPERATING_MODE_MEASURED_N_PARTIAL_CONTINUOUS = 0x2B;
    const uChar OPERATING_MODE_MEASURED_PER_SEGMENT_CONTINUOUS = 0x2C;
    const uChar OPERATING_MODE_NAV_DATA_RECORDS = 0x2E;
    const uChar OPERATING_MODE_ALL_MEASURED_PLUS_REFLECTIVITY_SUBRANGE_CONTINUOUS = 0x50;

    const uChar OPERATING_MODE_TEST_PASSWORD = 0x30;

    const uChar OPERATING_MODE_SET_BAUDRATE_38400  = 0x40;
    const uChar OPERATING_MODE_SET_BAUDRATE_19200  = 0x41;
    const uChar OPERATING_MODE_SET_BAUDRATE_9600   = 0x42;
    const uChar OPERATING_MODE_SET_BAUDRATE_500000 = 0x48;
    
    std::string operatingModeToString( uChar operatingMode );

    //////////////////////////////////////////////////////////////////////
    //
    // Operating Mode Response Data
    //
    const uChar OPERATING_MODE_RESPONSE_SUCCESS = 0x00;
    const uChar OPERATING_MODE_RESPONSE_FAIL    = 0x01;

    //////////////////////////////////////////////////////////////////////
    //
    // Status Response Data Stuff
    //
    const int VERSION_LENGTH = 6;
    const int SOFTWARE_VERSION_LENGTH = 6;
    const int MANUFACTURER_LENGTH = 8;
    const int POLLUTION_LENGTH = 8;
    const int REF_POLLUTION_LENGTH = 4;
    const int CALIB_POLLUTION_LENGTH = 8;
    const int CALIB_REF_POLLUTION_LENGTH = 4;

    //////////////////////////////////////////////////////////////////////
    //
    // Measuring Modes
    //
    const uChar MEASURING_MODE_8m80m_FIELD_AB_DAZZLE = 0x00; // (default value)
    const uChar MEASURING_MODE_8m80m_REFLECTOR8LEVELS = 0x01;
    const uChar MEASURING_MODE_8m80m_FIELD_ABC = 0x02;
    const uChar MEASURING_MODE_16m_REFLECTOR4LEVELS = 0x03;
    const uChar MEASURING_MODE_16m_FIELD_AB = 0x04;
    const uChar MEASURING_MODE_32m_FIELD_AB = 0x05;
    const uChar MEASURING_MODE_32m_FIELD_A = 0x06;

    std::string measuringModeToString( uChar mode );

    //////////////////////////////////////////////////////////////////////
    //
    // Status Codes
    //
    
    const uChar STATUS_GENERAL_MASK = 0x07;
    // These apply to the lower 3 bits as masked by STATUS_GENERAL_MASK
    const uChar STATUS_OK          = 0x00;
    const uChar STATUS_INFO        = 0x01;
    const uChar STATUS_WARNING     = 0x02;
    const uChar STATUS_ERROR       = 0x03;
    const uChar STATUS_FATAL_ERROR = 0x04;

    // one-bit masks
    const uChar STATUS_RESTART_MASK = 0x20;
    const uChar STATUS_IMPLAUSIBLE_MEASURED_VALUES_MASK = 0x30;
    const uChar STATUS_POLLUTION_MASK = 0x80;

    std::string statusToString( uChar status );
    bool generalStatusIsWarn( uChar generalStatus );
    bool generalStatusIsError( uChar generalStatus );
    

    //////////////////////////////////////////////////////////////////////
    //
    // Status Codes
    //

    const uChar MODE_SWITCH_SUCCESS_OK = 0x00;
    const uChar MODE_SWITCH_FAIL_BAD_PASSWORD = 0x01;
    const uChar MODE_SWITCH_FAIL_LMS_FAULT = 0x02;

    std::string modeSwitchSuccessToString( uChar success );

    //////////////////////////////////////////////////////////////////////
    //
    // Measurement units
    //

    const uChar MEASURED_VALUE_UNIT_CM = 0x00;
    const uChar MEASURED_VALUE_UNIT_MM = 0x01;

    std::string measuredValueUnitToString( uChar unit );

    //////////////////////////////////////////////////////////////////////
    //
    // Angular Resolution and scanning angle
    //
    const uint16_t ANGULAR_RESOLUTION_1_0_DEG = 100;
    const uint16_t ANGULAR_RESOLUTION_0_5_DEG = 50;
    const uint16_t ANGULAR_RESOLUTION_0_25_DEG = 25;

    inline double angularResolutionToDoubleInDegrees( uint16_t angularResolution )
    { return angularResolution/100.0; }

    const uint16_t SCANNING_ANGLE_180 = 180;
    const uint16_t SCANNING_ANGLE_100 = 100;

    const uChar SWITCH_VARIANT_SUCCESS = 0x01;
    const uChar SWITCH_VARIANT_FAIL    = 0x00;

    std::string switchVariantSuccessToString( uChar success );

    //////////////////////////////////////////////////////////////////////
    //
    // Sensitivity
    //

    const uChar SENSITIVITY_STANDARD = 0x00; // between medium and high
    const uChar SENSITIVITY_MEDIUM = 0x01;
    const uChar SENSITIVITY_LOW = 0x02;
    const uChar SENSITIVITY_HIGH = 0x03;

    std::string sensitivityToString( uChar sens );

    //////////////////////////////////////////////////////////////////////
    //
    // Configuration success
    //

    const uChar CONFIGURATION_SUCCESS = 0x01;
    const uChar CONFIGURATION_FAIL    = 0x00;

    std::string configurationSuccessToString( uChar success );

    //////////////////////////////////////////////////////////////////////
    //
    // Baud rates
    //

    // returns -1 on error
    int baudRateToInt( uint16_t baudRate );
    uChar baudRateIntToOperatingMode( int baudRate );

    std::string permanentBaudRateToString( uChar permanentBaudRate );

    //////////////////////////////////////////////////////////////////////
    //
    // Error codes
    //

    const uChar ERROR_TYPE_NO_LONGER_RELEVANT_MASK = 0x80;

    const uChar ERROR_CODE_DAZZLE_TEST = 05;
    const uChar ERROR_CODE_PEAK_COMPARATOR_TEST = 06;
    const uChar ERROR_CODE_STOP_COMPARATOR_TEST = 07;
    const uChar ERROR_CODE_TDC_INIT_AND_GATE_FUNCTION_TEST = 15;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_1 = 17;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_1 = 18;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_DIRT_CHANNEL_2 = 19;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_OIL_CHANNEL_2 = 20;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_0 = 21;
    const uChar ERROR_CODE_POLLUTION_EVALUATION_REF_CHANNEL_1 = 22;
    const uChar ERROR_CODE_OUTPUT_A_DEFECTIVE = 27;
    const uChar ERROR_CODE_OUTPUT_B_DEFECTIVE = 28;
    const uChar ERROR_CODE_NUM_MOTOR_REVOLUTIONS = 29;
    const uChar ERROR_CODE_CALIBRATION_POLLUTION = 37;
    const uChar ERROR_CODE_TIMEOUT_ON_TDC_CALIBRATION = 39;
    const uChar ERROR_CODE_1_MEASUREMENT_VALUE_MISSING = 45;
    const uChar ERROR_CODE_1_SCAN_MISSING_SCAN_LASTS_TOO_LONG = 46;
    const uChar ERROR_CODE_REFERENCE_TARGET_LOAD_PULSE_WIDTH_IMPLAUSIBLE = 47;
    const uChar ERROR_CODE_CALIBRATION_OF_LASER_POWER = 48;
    const uChar ERROR_CODE_LASER_POWER_OUTSIDE_50_PCT_140_PCT = 49;
    const uChar ERROR_CODE_INITIALISATION_TDC_MO_CHANNEL_0_AND_1 = 50;
    const uChar ERROR_CODE_DA_AD_TEST_STOP_BRANCH = 51;
    const uChar ERROR_CODE_DA_AD_TEST_PEAK_BRANCH = 52;
    const uChar ERROR_CODE_FLASH_WRITTEN = 53;
    const uChar ERROR_CODE_POLLUTION_CHANNEL_MEASUREMENT_WITHOUT_ACTIVE_TRANSMITTER = 54;
    const uChar ERROR_CODE_NO_TWO_DIFFERENT_ANGLES_DETECTED_ON_LASER_POWER_CALIBRATION = 55;
    const uChar ERROR_CODE_HARDWARE_WATCHDOG_DEFECTIVE = 56;
    const uChar ERROR_CODE_NO_ZERO_INDEX_SIGNAL_AVAILABLE = 57;
    const uChar ERROR_CODE_SLAVE_CANNOT_SYNC_DURING_INIT = 58;
    const uChar ERROR_CODE_SYNC_LOST_IN_OPERATING_STATE = 59;
    const uChar ERROR_CODE_SYNC_CYCLE_FROM_MASTER_MISSING = 60;
    const uChar ERROR_CODE_HARDWARE_UNSUITABLE_FOR_SYNC_IN_SLAVE_OPERATING_MODE = 61;
    const uChar ERROR_CODE_WRONG_DIP_SWITCH_POSITION = 62;
    const uChar ERROR_CODE_REF_TARGET_SMALLEST_PULSE_WIDTH_TOO_SMALL = 86;
    const uChar ERROR_CODE_REF_TARGET_LARGEST_PULSE_WIDTH_TOO_LARGE = 87;
    const uChar ERROR_CODE_REF_TARGET_PULSE_WIDTH_SPECTRUM_TOO_LARGE = 88;
    const uChar ERROR_CODE_REF_TARGET_REF_TABLE_LESS_THAN_2_CYCLES_UPDATE = 89;
    const uChar ERROR_CODE_REF_TARGET_REFLECTIVITY_MEASUREMENT_CANNOT_BE_CALIBRATED = 91;
    const uChar ERROR_CODE_REF_TARGET_TEACH_IN_MODE_IS_NOT_COMPLETED = 92;
    const uChar ERROR_CODE_OUT_OF_MEMORY_IN_MEASUREMENT_ROUTINE = 124;
    const uChar ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ROUTINE = 125;
    const uChar ERROR_CODE_OUT_OF_MEMORY_IN_REFERENCE_TARGET_ANGULAR_TABLE = 126;

    std::string errorTypeToString( uChar errorType );
    bool errorTypeIsWarn( uChar errorType );
    bool errorTypeIsError( uChar errorType );
    std::string errorCodeToString( uChar errorCode );
}

#endif
