/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2008-2010 Geoffrey Biggs
 *
 * hokuyo_aist Hokuyo laser scanner driver.
 *
 * This distribution is licensed to you under the terms described in the
 * LICENSE file included in this distribution.
 *
 * This work is a product of the National Institute of Advanced Industrial
 * Science and Technology, Japan. Registration number: H22PRO-1086.
 *
 * This file is part of hokuyo_aist.
 *
 * This software is licensed under the Eclipse Public License -v 1.0 (EPL). See
 * http://www.opensource.org/licenses/eclipse-1.0.txt
 */

#ifndef SENSOR_H__
#define SENSOR_H__

#include <string>

#if defined(WIN32)
    typedef unsigned char           uint8_t;
    typedef unsigned int            uint32_t;
    #if defined(HOKUYO_AIST_STATIC)
        #define HOKUYO_AIST_EXPORT
    #elif defined(HOKUYO_AIST_EXPORTS)
        #define HOKUYO_AIST_EXPORT       __declspec(dllexport)
    #else
        #define HOKUYO_AIST_EXPORT       __declspec(dllimport)
    #endif
#else
    #include <stdint.h>
    #define HOKUYO_AIST_EXPORT
#endif

namespace flexiport
{
    class Port;
}

/** @ingroup gbx_library_hokuyo_aist
@{
*/

namespace hokuyo_aist
{

/// @brief Possible values of the multiecho mode setting.
///
/// The Tough-URG features multiecho detection capability. To use this, set
/// the sensor to use any mode other than ME_OFF.
/// The sensor can register up to three echos for a single reading. The
/// multiecho mode determines how these are combined into a single value:
/// - ME_FRONT: only the closest reading will be used.
/// - ME_MIDDLE: the middle reading will be used, or the closest
///   reading if there are only two echos.
/// - ME_REAR: the furthest reading will be used.
/// - ME_AVERAGE: the average of all two or three echos will be used.
/// In all cases, if there is only one echo, then this setting has no effect.
enum MultiechoMode
{
    ME_OFF,
    ME_FRONT,
    ME_MIDDLE,
    ME_REAR,
    ME_AVERAGE
};


HOKUYO_AIST_EXPORT inline char const* multiecho_mode_to_string(MultiechoMode mode)
{
    switch(mode)
    {
        case ME_OFF:
            return "Off";
        case ME_FRONT:
            return "Front";
        case ME_MIDDLE:
            return "Middle";
        case ME_REAR:
            return "Rear";
        case ME_AVERAGE:
            return "Average";
        default:
            return "Unknown";
    }
}


/// Structure to store an IP address.
typedef struct IPAddr
{
    /// First byte
    unsigned int first;
    /// Second byte
    unsigned int second;
    /// Third byte
    unsigned int third;
    /// Fourth byte
    unsigned int fourth;
} IPAddr;


/** @brief Hokuyo laser scanner class.

Provides an interface for interacting with a Hokuyo laser scanner using SCIP
protocol version 1 or 2. The FlexiPort library is used to implement the data
communications with the scanner. See its documentation for details on
controlling the connection.

To use a serial connection, ensure that you do not also have a USB cable
connected, as this will force the scanner into USB mode, preventing the serial
connection from functioning correctly.

All functions may throw instances of @ref BaseError or its children.
Exceptions from @ref FlexiPort may also occur. */
class HOKUYO_AIST_EXPORT Sensor
{
    public:
        Sensor();
        Sensor(std::ostream& err_output);
        ~Sensor();

        /// @brief Open the laser scanner and begin scanning.
        void open(std::string port_options);

        /** @brief Open the laser scanner and begin scanning, probing the baud
        rate.

        If the port is a serial connection and communication with the laser
        fails at the given baud rate, the alternative baud rates supported by
        the device are tried (see @ref set_baud for these) in order from fastest
        to slowest.

        @return The baud rate at which connection with the laser succeeded, or
        0 for non-serial connections. */
        unsigned int open_with_probing(std::string port_options);

        /// @brief Close the connection to the laser scanner.
        void close();

        /// @brief Checks if the connection to the laser scanner is open.
        bool is_open() const;

        /// @brief Switch the laser scanner on or off.
        void set_power(bool on);

        /** @brief Change the baud rate when using a serial connection.

        Valid rates are 19.2Kbps, 38.4Kbps, 57.6Kbps, 115.2Kbps, 250.0Kbps,
        500.0Kbps, 750.0Kbps (dependent on those available in FlexiPort). */
        void set_baud(unsigned int baud);

        /** @brief Change the IP address information.

        This function only works on devices providing an ethernet connection.

        Once this function has successfully completed, the laser must be
        restarted for it to take effect. */
        void set_ip(IPAddr const& addr, IPAddr const& subnet,
                IPAddr const& gateway);

        /** @brief Reset the laser scanner to its default settings.

        Not available with the SCIP v1 protocol. */
        void reset();

        /** @brief Reset everything except motor and serial speed.

        Requires SCIP v2.1 or higher. */
        void semi_reset();

        /** @brief Set the speed at which the scanner's sensor spins.

        Set the speed to 0 to have it reset to the default value, and 99 to
        reset it to the initial (startup) value. Values between 1 and 10
        specify a ratio of the default speed. The speeds in revolutions per
        minute that these correspond to will depend on the scanner model. For
        example, for a URG-04LX, they are (from 1 to 10) 594, 588, 576, 570,
        564, 558, 552, 546, and 540 rpm.

        Not available with the SCIP v1 protocol. */
        void set_motor_speed(unsigned int speed);

        /** @brief Switch the scanner between normal and high sensitivity
        modes. */
        void set_high_sensitivity(bool on);

        /** @brief Get various information about the scanner.

        Much of the information is not available with the SCIP v1 protocol. */
        void get_sensor_info(SensorInfo& info);

        /** @brief Get the value of the scanner's clock in milliseconds.

        Not available with the SCIP v1 protocol. */
        unsigned int get_time();

        /** @brief Get the raw value of the scanner's clock in milliseconds.

        Not available with the SCIP v1 protocol. */
        unsigned int get_raw_time();

        /** @brief Calibrate the time offset between the laser and computer.

        This function performs several checks of the communications between the
        laser and the computer. Its goal is to calculate the communications lag
        between the two devices, and so determine as accurately as possible
        what the offset is from the computer's clock to the laser's clock. This
        offset is necessary to calculate the time stamp of scans in terms of
        the computer's clock.

        Because this function sends a lot of data to and from the laser, it
        may take some time to complete. It will never be called automatically;
        until it is called, the offset defaults to zero.

        Note that the laser's clock is a 24-bit millisecond timer. It wraps
        approximately every 9.5 hours. This class will detect wrapped time
        stamps and adjust them accordingly.

        @param skew_sleep_time Whether to approximate the laser clock skew.
        This requires sleeping for the specified time, then calibrating again.
        The results of the two calibrations are used to approximate the clock
        skew as a straight line between the two points. This is very
        approximate; in order to overcome noise in the signal you will need to
        calibrate over a period of up to several minutes. Set this to 0 to not
        approximate the skew.
        @param samples The number of samples to use in calculating latencies.
        @return The calculated offset in nanoseconds.
        */

        long long calibrate_time(unsigned int skew_sleep_time=0,
                unsigned int samples=10);

        /// Retrieve the calculated time offset (0 if not calibrated).
        long long time_offset() const { return time_offset_; }

        /// Set the time offset (if the calculated value is bad).
        void set_time_offset(long long time_offset)
            { time_offset_ = time_offset; }
        /// Retrieve the current clock drift rate (0 if not set).
        float drift_rate() const { return time_drift_rate_; }
        /** Set the current clock drift rate.

        The drift rate is used when correcting laser time stamps to computer
        time. It is factored into the equation as:
            Tc = ((1 - drift)Tl + offset + beta) / (1 - alpha)
        where Tc is the computer time and Tl is the laser time. See @ref
        set_skew_alpha for the values of alpha and beta.

        If the drift rate is zero, it means the laser's clock does not drift.
        This is extremely unlikely, but the laser's drift may not matter for
        your application.

        Drift should usually be provided by the manufacturer. If it is not,
        you can calculate it by calibrating the laser many times over a long
        period and looking at the change in the calculated offset. */
        void set_drift_rate(float drift_rate)
            { time_drift_rate_ = drift_rate; }

        /// Get the calculated skew line slope (default: 0).
        float skew_alpha() const { return time_skew_alpha_; }
        /** Set a skew line slope value.

        The skew is used when correcting laser time stamps to computer time.
        It is factored into the equation as:
            Tc = ((1 - drift)Tl + offset + beta) / (1 - alpha)
        where Tc is the computer time and Tl is the laser time. See @ref
        drift_rate for the value of the drift. Beta, the line's y crossing, is
        assumed to be 0 since we always use a two-point line fit instead of
        something like least squares.

        The effect of this is to cancel out the skew caused by the different
        frequencies of the computer clock and the laser clock. */
        void set_skew_alpha(float alpha) { time_skew_alpha_ = alpha; }

        /** @brief Get the latest scan data from the scanner.

        This function requires a pointer to a @ref ScanData object. It will
        allocate space in this object as necessary for storing range data. If
        the passed-in @ref ScanData object already has the correct quantity
        of space to store the range data, it will not be re-allocated. If it
        does not have any space, it will be allocated. If it has space, but it
        is the wrong size, it will be re-allocated. This means you can
        repeatedly send the same @ref ScanData object without having to worry
        about allocating its data, whether it will change or not, while also
        avoiding excessive allocations.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @param start_step The first step to get ranges from. Set to -1 for the
        first scannable step.
        @param end_step The last step to get ranges from. Set to -1 for the last
        scannable step.
        @return The number of range readings read into @ref data. */
        unsigned int get_ranges(ScanData& data, int start_step = -1,
                int end_step = -1, unsigned int cluster_count = 1);

        /** @brief Get the latest scan data from the scanner.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param start_angle The angle to get range readings from. Exclusive; if
        this falls between two steps the step inside the angle will be
        returned, but the step outside won't.
        @param end_angle The angle to get range readings to. Exclusive; if this
        falls between two steps the step inside the angle will be returned, but
        the step outside won't.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @return The number of range readings read into @ref data. */
        unsigned int get_ranges_by_angle(ScanData& data, double start_angle,
                double end_angle, unsigned int cluster_count = 1);

        /** @brief Get the latest scan data from the scanner with intensities.

        This function requires a pointer to a @ref ScanData object. It will
        allocate space in this object as necessary for storing range data. If
        the passed-in @ref ScanData object already has the correct quantity
        of space to store the range data, it will not be re-allocated. If it
        does not have any space, it will be allocated. If it has space, but it
        is the wrong size, it will be re-allocated. This means you can
        repeatedly send the same @ref ScanData object without having to worry
        about allocating its data, whether it will change or not, while also
        avoiding excessive allocations.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @param start_step The first step to get ranges from. Set to -1 for the
        first scannable step.
        @param end_step The last step to get ranges from. Set to -1 for the last
        scannable step.
        @return The number of range readings read into @ref data. */
        unsigned int get_ranges_intensities(ScanData& data,
                int start_step = -1, int end_step = -1,
                unsigned int cluster_count = 1);

        /** @brief Get the latest scan data from the scanner with intensities.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param start_angle The angle to get range readings from. Exclusive; if
        this falls between two steps the step inside the angle will be
        returned, but the step outside won't.
        @param end_angle The angle to get range readings to. Exclusive; if this
        falls between two steps the step inside the angle will be returned, but
        the step outside won't.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @return The number of range readings read into @ref data. */
        unsigned int get_ranges_intensities_by_angle(ScanData& data,
                double start_angle, double end_angle,
                unsigned int cluster_count = 1);

        /** @brief Get a new scan from the scanner.

        Unlike @ref get_ranges, which returns the most recent scan the scanner
        took, this function will request a new scan. This means it will wait
        while the scanner performs the scan, which means the rate at which
        scans can be retrieved using this function is less than with @ref
        get_ranges. Otherwise behaves identicallty to @ref get_ranges.

        Not available with the SCIP v1 protocol.

        @note The command used to retrieve a fresh scan is also used for the
        continuous scanning mode (not yet supported by this library). After
        completing a scan, it will turn the laser off (in anticipation of
        another continuous scan command being sent, which will automatically
        turn the laser back on again). If you want to mix @ref get_new_ranges and
        @ref get_ranges, you will need to turn the laser on after each call to
        @ref get_new_ranges.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @param start_step The first step to get ranges from. Set to -1 for the
        first scannable step.
        @param end_step The last step to get ranges from. Set to -1 for the last
        scannable step.
        @return The number of range readings read into @ref data. */
        unsigned int get_new_ranges(ScanData& data, int start_step = -1,
                int end_step = -1, unsigned int cluster_count = 1);

        /** @brief Get a new scan from the scanner.

        Not available with the SCIP v1 protocol.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param start_angle The angle to get range readings from. Exclusive; if
        this falls between two steps the step inside the angle will be
        returned, but the step outside won't.
        @param end_angle The angle to get range readings to. Exclusive; if this
        falls between two steps the step inside the angle will be returned, but
        the step outside won't.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @return The number of range readings read into @ref data. */
        unsigned int get_new_ranges_by_angle(ScanData& data,
                double start_angle, double end_angle,
                unsigned int cluster_count = 1);

        /** @brief Get a new scan from the scanner with intensity data.

        Unlike @ref get_ranges, which returns the most recent scan the scanner
        took, this function will request a new scan. This means it will wait
        while the scanner performs the scan.  Otherwise behaves identicallty to
        @ref get_ranges.

        Not available with the SCIP v1 protocol.

        @note The command used to retrieve a fresh scan is also used for the
        continuous scanning mode (not yet supported by this library). After
        completing a scan, it will turn the laser off (in anticipation of
        another continuous scan command being sent, which will automatically
        turn the laser back on again). If you want to mix @ref get_new_ranges and
        @ref get_ranges, you will need to turn the laser on after each call to
        @ref get_new_ranges.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @param start_step The first step to get ranges from. Set to -1 for the
        first scannable step.
        @param end_step The last step to get ranges from. Set to -1 for the last
        scannable step.
        @return The number of range readings read into @ref data. */
        unsigned int get_new_ranges_intensities(ScanData& data,
                int start_step = -1, int end_step = -1,
                unsigned int cluster_count = 1);

        /** @brief Get a new scan from the scanner with intensity data.

        Not available with the SCIP v1 protocol.

        @param data Pointer to a @ref ScanData object to store the range
        readings in.
        @param start_angle The angle to get range readings from. Exclusive; if
        this falls between two steps the step inside the angle will be
        returned, but the step outside won't.
        @param end_angle The angle to get range readings to. Exclusive; if this
        falls between two steps the step inside the angle will be returned, but
        the step outside won't.
        @param cluster_count The number of readings to cluster together into a
        single reading. The minimum value from a cluster is returned as the
        range for that cluster.
        @return The number of range readings read into @ref data. */
        unsigned int get_new_ranges_intensities_by_angle(ScanData& data,
                double start_angle, double end_angle,
                unsigned int cluster_count = 1);

        /// @brief Return the major version of the SCIP protocol in use.
        uint8_t scip_version() const            { return scip_version_; }

        /** @brief Turns on and off printing of verbose operating information
        to stderr. Default is off. */
        void set_verbose(bool verbose)          { verbose_ = verbose; }

        /** @brief Enables/disables ignoring unknown lines in sensor info
        messages. Default is off. */
        void ignore_unknowns(bool ignore)       { ignore_unknowns_ = ignore; }

        /** @brief Set the multi-echo mode to use. Default is ME_OFF. */
        void set_multiecho_mode(MultiechoMode mode) { multiecho_mode_ = mode; }

        /// @brief A convenience function to convert a step index to an angle.
        double step_to_angle(unsigned int step);
        /** @brief A convenience function to convert an angle to a step
        (rounded towards the front). */
        unsigned int angle_to_step(double angle);

    private:
        flexiport::Port* port_;
        std::ostream& err_output_;

        uint8_t scip_version_;
        LaserModel model_;
        bool verbose_, enable_checksum_workaround_,
             ignore_unknowns_;
        MultiechoMode multiecho_mode_;
        double min_angle_, max_angle_, resolution_;
        int first_step_, last_step_, front_step_;
        unsigned int max_range_;
        /// The time between two points in a scan, in milliseconds.
        unsigned int time_resolution_;
        /// The offset from the laser's clock to the computer's clock in
        /// nanoseconds.
        long long time_offset_;
        /// The previous received timestamp from the laser, in laser time and
        /// in milliseconds.
        unsigned int last_timestamp_;
        /// The number of times the laser's clock has wrapped.
        unsigned int wrap_count_;
        /// The drift rate of the laser's clock
        float time_drift_rate_;
        /// The clock skew alpha value.
        float time_skew_alpha_;

        void clear_read_buffer();
        int read_line(char* buffer, int expected_length=-1);
        int read_line_with_check(char* buffer, int expected_length=-1,
                bool has_semicolon=false);
        bool read_data_block(char* buffer, int& block_size);
        void skip_lines(int count);
        int send_command(char const* cmd, char const* param, int param_length,
                char const* extra_ok);

        void enter_timing_mode();
        void leave_timing_mode();
        /// Get the laser time in milliseconds.
        unsigned int get_timing_mode_time(unsigned long long* reception_time=0);
        /// Get the computer time in nanoseconds.
        unsigned long long get_computer_time();
        /// Adjust a wrapped laser timestamp, in milliseconds.
        unsigned int wrap_timestamp(unsigned int timestamp);
        /// Offset a laser timestamp, in milliseconds, into computer time, in
        /// nanoseconds.
        unsigned long long offset_timestamp(unsigned int timestamp);
        /// Convert a step value into its time offset from the start of a scan,
        /// in milliseconds.
        unsigned int step_to_time_offset(int start_step);

        void find_model(char const* buffer);
        void get_and_set_scip_version();
        void get_defaults();
        void process_vv_line(char const* buffer, SensorInfo& info);
        void process_pp_line(char const* buffer, SensorInfo& info);
        void process_ii_line(char const* buffer, SensorInfo& info);

        uint32_t process_echo_buffer(int const* buffer, int num_echos);
        void read_2_byte_range_data(ScanData& data, unsigned int num_steps);
        void read_3_byte_range_data(ScanData& data, unsigned int num_steps);
        void read_3_byte_range_and_intensity_data(ScanData& data,
                unsigned int num_steps);

        int confirm_checksum(char const* buffer, int length,
                int expected_sum);
}; // class Sensor

} // namespace hokuyo_aist

/** @} */

#endif // SENSOR_H__

