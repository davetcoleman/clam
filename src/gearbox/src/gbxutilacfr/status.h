/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_STATUS_H
#define GBXUTILACFR_STATUS_H

#if defined (WIN32)
    #if defined (GBXUTILACFR_STATIC)
        #define GBXUTILACFR_EXPORT
    #elif defined (GBXUTILACFR_EXPORTS)
        #define GBXUTILACFR_EXPORT       __declspec (dllexport)
    #else
        #define GBXUTILACFR_EXPORT       __declspec (dllimport)
    #endif
#else
    #define GBXUTILACFR_EXPORT
#endif

#include <string>
#include <vector>

namespace gbxutilacfr {

//! Possible subsystem status values
enum SubsystemState
{
    //! Subsystem has been created but has not started initialisation process.
    SubsystemIdle,
    //! Subsystem is preparing to work, e.g. initialising its resources, etc.
    SubsystemInitialising,
    //! Subsystem is fully initialised and is performing its function.
    SubsystemWorking,
    //! Subsystem is preparing to shutdown, e.g. releasing its resources, etc.
    SubsystemFinalising,
    //! Subsystem is not longer functioning.
    SubsystemShutdown,
    //! Subsystem is in an unrecovarable faulty state.
    SubsystemFault
};

//! Returns string equivalent of state enumerator.
GBXUTILACFR_EXPORT std::string toString( SubsystemState state );

//! Possible subsystem status values
enum SubsystemHealth
{
    //! Subsystem is OK
    SubsystemOk,
    //! Subsystem has encountered an abnormal but non-critical condition
    SubsystemWarning,
    //! Subsystem has encountered a critical condition
    SubsystemCritical
};

//! Returns string equivalent of health enumerator.
GBXUTILACFR_EXPORT std::string toString( SubsystemHealth health );

//! Status for a single subsystem
struct GBXUTILACFR_EXPORT SubsystemStatus
{
    //! Constructor.
    SubsystemStatus( SubsystemState s=SubsystemIdle, SubsystemHealth h=SubsystemOk, const std::string& msg="",
                     bool stall=false, double beat=0.0 ) :
        state(s),
        health(h),
        message(msg),
        isStalled(stall),
        sinceHeartbeat(beat) {};

    //! Current state in the subsystem's state machine. I.e. what is the subsystem doing?
    SubsystemState state;

    //! Subsystem's health. I.e. how is the subsystem doing?
    SubsystemHealth health;

    //! Human-readable status description
    std::string message;

    //! If true, the subsystem has not been heard from for an abnormally long time.
    bool isStalled;

    //! OBSOLETE !?
    //! Ratio of time since last heartbeat to maximum expected time between heartbeats.
    //! For example, sinceHeartbeat=0.5 means that half of normally expected interval between heartbeats
    //! has elapsed.
    double sinceHeartbeat;
};

//! Returns human-readable string with subsystem status information.
GBXUTILACFR_EXPORT std::string toString( const SubsystemStatus& status );

//! Subsystem type which describes common behavior models of a subsystem.
enum SubsystemType
{
    //! Standard model: subsystem's life cycle is equal to the life cycle of the component.
    SubsystemStandard,
    //! Early exit model: subsystem's life cycle is shorter than the life cycle of the component.
    //! This model is used for two common cases:
    //! - subsystem performs some function and intentionally shuts down early;
    //! - subsystem starts up, works, encounters a problem, shuts down, and restarts again.
    SubsystemEarlyExit,
    //! Internal model. May shutdown early (same as SubsystemEarlyExit). In addition,
    //! this subsystem is not considered the "real" worker subsystem, i.e. when it
    //! is the only subsystem in existance, the component is not considered operational
    //! even if this subsystem is working.
    SubsystemInfrastructure
};

//! Returns string equivalent of subsystem type enumerator.
GBXUTILACFR_EXPORT std::string toString( SubsystemType type );

//! Possible component states.
enum ComponentState
{
    //! Component is preparing to work, e.g. initialising its resources, etc.
    CompStarting,
    //! Component is fully initialised and is performing its work
    CompOperational,
    //! Component is preparing to shutdown, e.g. releasing its resources, etc.
    CompStopping,
    //! Component is in an unrecovarable faulty state.
    CompFault
};

//! Returns string equivalent of component state type enumerator.
GBXUTILACFR_EXPORT std::string toString( ComponentState type );

//! Possible values of component health.
enum ComponentHealth
{
    //! All of the component's subsystems are OK.
    CompOk,
    //! At least one of the component's subsystems has encountered an abnormal but non-critical condition.
    CompWarning,
    //!  At least one of the component's subsystems has encountered a critical condition.
    CompCritical
};

//! Returns string equivalent of component health type enumerator.
GBXUTILACFR_EXPORT std::string toString( ComponentHealth type );

//! Status of a single component.
struct GBXUTILACFR_EXPORT ComponentStatus
{
    //! Component state
    ComponentState state;
    //! Component health
    ComponentHealth health;
    //! Is component stalled?
    bool isStalled;
};

/*!
@brief Local interface to component status.

@par Overview

Status provides a machine-readable interface such that tools external
to the component can monitor its status. A single Status object is meant
to be shared by all threads in the library, so the implementation must
be thread-safe. The idea is that Status tracks the state of a number
of subsystems (most often one per thread).

Each subsystem should first call addSubsystem(), to make the
Status engine aware that it exists. If any other function is called before
the subsystem is added, a gbxutilacfr::Exception is thrown.

The default initial status of a subsystem is @c Idle with health @c OK.

After registering a subsystem, a subsystem can report its state and health.
Each of the calls is sufficient to let the Status engine know that the subsystem is alive.
The special call heartbeat() lets Status know that the subsystem is alive without
modifying its status.

The 'maxHeartbeatIntervalSec' parameter tells the Status engine how often it
should expect to hear from the subsystem.  If no message is received from a subsystem for
longer than @c maxHeartbeatIntervalSec, it is assumed that the subsystem has stalled (hung).

@par State Machine

The state machine of a subsystem is a chain of state transitions with one extra link:
@verbatim
Idle --> Initialising --> Working --> Finalising --> Shutdown
              |___________________________^
@endverbatim
In addition, the system can transition to Fault state from any other state.

The following represents the Subsystem state machine in the format of
State Machine Compiler (see smc.sf.net) :
@verbatim
Idle
Entry { init(); }
{
    init
        Initialising
        {}
}

Initialising
Entry { initialise(); }
{
    [ !isStopping ] finished
        Working
        {}

    [ isStopping ] finished
        Finalising
        {}
}

Working
Entry { work(); }
{
    finished
        Finalising
        {}
}

Finalising
Entry { finalise(); }
{
    finished
        Shutdown
        {}
}

Shutdown
{
}

Fault
{
}

Default
{
    fault
        Fault
        {}
}
@endverbatim

@sa Tracer
@sa SubStatus
*/
class GBXUTILACFR_EXPORT Status
{

public:

    virtual ~Status() {};

    /*!
    Adds a new subsystem to the system status descriptor. This command must be called before actually
    modifying the subsystem status, i.e. all other status commands will raise an exception if a subsystem with
    that name does not already exists.

    An Exception is also raised when trying to add a subsystem with an existing name.

    It is possible to specify the maximum expected interval between heartbeats. See setMaxHeartbeatInterval()
    for details.

    It is also possible to describe the expected behavior of the subsystem by specifying SubsystemType. See
    setSubsystemType() for details.

    The initial status of the new subsystem is the same as produced by the empty constructor of SubsystemStatus.
    */
    virtual void addSubsystem( const std::string& subsystem,
            double maxHeartbeatIntervalSec=-1.0, SubsystemType type=SubsystemStandard )=0;

    //! Removes a subsystem from the status descriptor.
    //! Throws Exception if the subsystem does not exist.
    virtual void removeSubsystem( const std::string& subsystem )=0;

    //! Returns a list of subsystem names.
    virtual std::vector<std::string> subsystems()=0;

    //! Returns status of the subsystem with the given name.
    //! Throws Exception when the specified subsystem does not exist.
    virtual SubsystemStatus subsystemStatus( const std::string& subsystem )=0;

    //! Returns state of the component infrastructure.
//     virtual SubsystemState infrastructureState()=0;

    //! Returns component status.
    //! Component status cannot be set directly, it is the aggregate of the
    //! states and healths of its subsystems and the infrastructure.
    virtual ComponentStatus componentStatus()=0;

    //! Sets the maximum expected interval between heartbeats (in seconds).
    //! When time since the last heartbeat exceeds the specified value, the subsystem is considered stalled.
    //! Negative interval means infinite interval.
    //! Throws Exception if the subsystem does not exist.
    virtual void setMaxHeartbeatInterval( const std::string& subsystem, double intervalSec )=0;

    //! Sets the subsystem type which describes the expected behavior of the subsystem.
    virtual void setSubsystemType( const std::string& subsystem, SubsystemType type )=0;

    //
    // BOTH STATE AND HEALTH CHANGES
    //

    //! Sets the status of a subsystem (both state and health) in an atomic operation. Use this method
    //! when both state and health have changed.
    //! Throws Exception if the subsystem does not exist.
    virtual void setSubsystemStatus( const std::string& subsystem, SubsystemState state, SubsystemHealth health, const std::string& message="" )=0;

    //
    // STATE CHANGES
    //

    //! Sets state of the subsystem to Initialising. Health and message are not affected.
    //! Throws Exception if the subsystem does not exist.
    virtual void initialising( const std::string& subsystem )=0;

    //! Sets state of the subsystem to Working. Health and message are not affected.
    //! Throws Exception if the subsystem does not exist.
    virtual void working( const std::string& subsystem )=0;

    //! Sets state of the subsystem to Finalising. Health and message are not affected.
    //! Throws Exception if the subsystem does not exist.
    virtual void finalising( const std::string& subsystem )=0;

    //! Sets state of the subsystem to Fault. Diagnostic message is required.
    //! Subsystem health is automatically changed to Critical.
    //! Throws Exception if the subsystem does not exist.
    virtual void fault( const std::string& subsystem, const std::string& message )=0;

    //
    // HEALTH CHANGES
    //

    //! Sets subsystem health to Ok. The old message is cleared if a new one is not supplied.
    //! Throws Exception if the subsystem does not exist.
    virtual void ok( const std::string& subsystem, const std::string& message="" )=0;

    //! Sets subsystem health to Warning. Diagnostic message is required.
    //! Throws Exception if the subsystem does not exist.
    virtual void warning( const std::string& subsystem, const std::string& message )=0;

    //! Sets subsystem health to Critical. Diagnostic message is required.
    //! Throws Exception if the subsystem does not exist.
    virtual void critical( const std::string& subsystem, const std::string& message )=0;

    //
    // NO CHANGE
    //

    //! Record heartbeat from a subsystem: let Status know the subsystem is alive without
    //! modifying its status.
    //! Throws Exception if the subsystem does not exist.
    virtual void heartbeat( const std::string& subsystem )=0;

    //! Change the human-readable message for a subsystem but keep the previous state and health information.
    //! Throws Exception if the subsystem does not exist.
    virtual void message( const std::string& subsystem, const std::string& message )=0;

    //
    // Utility
    //

    //! This function must be called periodically in order for
    //! status publishing to happen and stalled susbsystems identified.
    virtual void process()=0;
};

} // namespace

#endif
