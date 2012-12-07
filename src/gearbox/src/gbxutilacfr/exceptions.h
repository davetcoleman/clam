/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXUTILACFR_EXCEPTIONS_H
#define GBXUTILACFR_EXCEPTIONS_H

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

/*
 * STRINGIZE macro converts an expression into a string-literal.
 * ERROR_INFO macro permits file-name and line-number data to be added to an error message.
 *
 * Adapted by Alex Brooks from Tim Bailey's version 2005.
 */

#ifndef ERROR_MACROS_HPP_
#define ERROR_MACROS_HPP_

#if defined(STRINGIZE_HELPER) || defined(STRINGIZE) || defined(ERROR_INFO)
#   error GbxUtilAcfr error macros have already been defined elsewhere 
#endif

#define STRINGIZE_HELPER(exp) #exp
#define STRINGIZE(exp) STRINGIZE_HELPER(exp)

#define ERROR_INFO __FILE__, STRINGIZE(__LINE__)

#endif

#include <exception>
#include <string>

namespace gbxutilacfr {

/*!
@brief Base class for all GbxUtilAcfr exceptions.

Can be caught as a std::exception due to inheritance.

Generally, this exception should be thrown like so:

@verbatim
throw gbxutilacfr::Exception( ERROR_INFO, "description of error" );
@endverbatim

where the ERROR_INFO macro inserts the offending file and line number.
*/
class GBXUTILACFR_EXPORT Exception : public std::exception
{
public:
    Exception(const char *file, const char *line, const std::string &message);

    virtual ~Exception() throw();

    virtual const char* what() const throw() { return message_.c_str(); }

protected:
    std::string toMessageString( const char *file, const char *line, const std::string &message ) const;

    std::string  message_;
};
std::string basename( const std::string &s );
std::string dirname( const std::string &s );

//! This exception is raised when something is wrong with the hardware.
class GBXUTILACFR_EXPORT HardwareException : public gbxutilacfr::Exception
{
public:
    HardwareException(const char *file, const char *line, const std::string &message)
            : Exception( file, line, message ) {};
};

} // namespace


#endif
