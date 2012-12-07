/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef GBXSERIALACFR_LOCKFILE_H
#define GBXSERIALACFR_LOCKFILE_H

#include <exception>
#include <string>

namespace gbxserialacfr {
namespace lockfile {

//!
//! @brief Exception thrown by lockfile functions
//!
class LockFileException : public std::exception
{ 
    std::string  message_;
public:
    LockFileException( const std::string &message )
        : message_(message) {}
    ~LockFileException()throw(){}
    virtual const char* what() const throw() { return message_.c_str(); }
};

//!
//! @brief Thrown when we try to lock a resource which has been locked by another process.
//!
class LockedByOtherProcessException : public LockFileException
{
public:
    LockedByOtherProcessException( const std::string &message )
        : LockFileException(message) {}
    ~LockedByOtherProcessException()throw(){}
};

//!
//! Creates a lock-file (in /var/lock) which can be used to prevent multiple access to
//! a unique resource (eg "/dev/xxx").
//! Stores the PID of the locking process (lockPid), so it can check for stale lock-files.
//!
//! Throws LockFileException's on errors (including 'device locked').
//!
//! The destructor removes the lock-file (guarantees that no exceptions are thrown).
//!
//! For more info, see: http://tldp.org/HOWTO/Serial-HOWTO-14.html
//!
//! @author Alex Brooks
//!
class LockFile {
public:

    LockFile( const std::string &dev,
              int lockPid = getpid() );
    ~LockFile();

private:

    const std::string dev_;
    const int lockPid_;

};

}
}
#endif
