/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXICEUTILACFR_NOTIFY_H
#define GBXICEUTILACFR_NOTIFY_H

#include <gbxutilacfr/exceptions.h>
#include <iostream>

//
// note: this class can be libGbxUtilAcfr but we keep it with the other "data pattern"
// classes: Store and Buffer.
//
namespace gbxiceutilacfr {

/*!
 *  @brief The object which implements the callback function.
 *
 *  Derive from this class and implement the callback function NotifyHandler::handleData and
 *  register it with Notify by calline Notify::setNotifyHandler.
 */
template<class Type>
class NotifyHandler
{
public:
    virtual ~NotifyHandler() {};
    //!
    //! This function must be implemented by the component developer.
    //!
    virtual void handleData( const Type & obj )=0;
};

/*!
 * @brief A data pipe with callback semantics.
 *
 *  Write new data with Notify::set. The data is delivered to the data handler by
 *  calling NotifyHandler::handleData in the registered NotifyHandler.
 *
 *  When used with smart pointers (e.g. IceUtil smart pointers), this class will not 
 *  perform a deep copy.
 *
 *  @see Buffer, Proxy
 */
template<class Type>
class Notify
{
public:
    Notify()
        : hasNotifyHandler_(false)
    {};

    virtual ~Notify() {};

    //! Sets internal link to the notify handler. If the provided pointer is NULL,
    //! the internal link is quietly not set.
    void  setNotifyHandler( NotifyHandler<Type>* handler );

    //! Returns TRUE is the notify handler has been set and FALSE otherwise.
    bool hasNotifyHandler() { return hasNotifyHandler_; };

    //! Forwards the @p obj to the data handler.
    //! Raises gbxutilacfr::Exception if the function is called when a notify handler has
    //! not been set.
    void set( const Type & obj );

protected:
    //! Reimplement this function for non-standard types.
    virtual void internalSet( const Type & obj );
    
    //! Interface to the object which is notified of incoming data.
    NotifyHandler<Type>* handler_;
    
private:

    bool hasNotifyHandler_;
};

template<class Type>
void Notify<Type>::setNotifyHandler( NotifyHandler<Type>* handler )
{
    if ( handler == 0 ) {
        std::cout<<"TRACE(notify.h): no handler set.  Ignoring data." << std::endl;
        return;
    }
    
    handler_ = handler;
    hasNotifyHandler_ = true;
}

template<class Type>
void Notify<Type>::set( const Type & obj )
{
    if ( !hasNotifyHandler_ ) {
        throw gbxutilacfr::Exception( ERROR_INFO, "setting data when data handler has not been set" );
    }

    internalSet( obj );
}

template<class Type>    
void Notify<Type>::internalSet( const Type & obj )
{
    handler_->handleData( obj );
}

} // end namespace

#endif
