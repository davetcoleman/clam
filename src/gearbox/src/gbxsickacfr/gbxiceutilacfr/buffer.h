/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXICEUTILACFR_BUFFER_H
#define GBXICEUTILACFR_BUFFER_H

#include <queue>
#include <gbxutilacfr/exceptions.h>

#include <IceUtil/Monitor.h>
#include <IceUtil/Mutex.h>
#include <IceUtil/Time.h>

namespace gbxiceutilacfr {

//! %Buffer type defines behavior when the buffer is full
enum BufferType
{
    //! When the maximum depth of the buffer is reached, removes the oldest entry to
    //! make room for the new data.
    BufferTypeCircular,
    //! When the maximum depth of the buffer is reached, ignores new incoming data,
    //! until the oldest entry is popped to make room.
    BufferTypeQueue
};

/*!
@brief A thread-safe data pipe with buffer semantics.

For a type-safe buffer, template over the specific object Type you want to put in it. You can use 
this container for storing smart pointers (e.g. IceUtil smart pointers). In this case the container will
only store the pointers and will not perform a deep copy.

@note This implementation uses IceUtil threading classes. See example in sec. 28.9.2 of the Ice manual.
@see Notify, Proxy
*/
template<class Type>
class Buffer : public IceUtil::Monitor<IceUtil::Mutex>
{
public:

    /*!
     *  Buffer depth, i.e. the maximum number of objects this buffer can hold:
     *      - positive numbers to specify finite depth,
     *      - negative numbers for infinite depth, limited by memory size
     *      - zero is undefined
     */
    Buffer( int depth = -1, BufferType type = BufferTypeQueue );

    virtual ~Buffer();

    //! Typically is called before the buffer is used, or if, for some reason, the configuration
    //! information was not available at the time when the constructor was called.
    //! Careful: all data currently in the buffer is lost, because @ref purge() is calledfirst.
    //! NOTE: can do smarter by trancating queue only as much as needed.
    void configure( int depth, BufferType type=BufferTypeCircular );

    //! Returns buffer depth.
    int depth() const;

    //! Returns buffer type.
    BufferType type() const;

    //! Returns FALSE if there's something in the buffer.
    bool isEmpty() const;

    //! Returns the number of items in the buffer.
    int  size() const;

    //! Deletes all entries, makes the buffer empty.
    void purge();

    /*!
     *  Adds an object to the end of the buffer. If there is no room left in a finite-depth
     *  circular buffer, the front element of the buffer is quietly deleted and the new data
     *  is added to the end. If there is no room left in a finite-depth queue buffer
     *  the new data is quietly ignored.
     */
    void push( const Type & obj );

    /*! 
     *  Pops the front element off and discards it (usually after calling @ref get() ).
     *  If the buffer is empty this command is quietly ignored.
     */
    void pop();

    /*!
     *  Non-popping and non-blocking read from the front of the buffer.
     *
     *  Calls to @ref get() on an empty buffer raises an gbxutilacfr::Exception exception.
     *  You can catch these and call @ref getWithTimeout() which will block until new data arrives.
     */
    void  get( Type & obj ) const;

    /*!
     *  Non-popping and non-blocking random-access read. Returns n-th element from the buffer.
     *  Indexing starts at 0.
     */
    void  get( Type & obj, unsigned int n ) const;

    /*!
     *  Same as @ref get() but calls @ref pop() afterwards.
     */
    void  getAndPop( Type & obj );

    /*! 
     *  If there is an object in the buffer, sets the object and returns 0;
     *
     *  If the buffer is empty, @ref getWithTimeout() blocks until a new object is pushed in 
     *  and returns the new value. By default, there is an infinite timeout (negative value). 
     *  Returns 0 if successful.
     *
     *  If timeout is set to a positive value and the wait times out, this function returns -1
     *  and the object argument itself is not touched. In the rare event of spurious wakeup,
     *  the return value is 1.
     */
    int  getWithTimeout( Type & obj, int timeoutMs=-1 );

    /*!
     *  Same as @ref getWithTimeout but calls @ref pop afterwards.
     */
    int  getAndPopWithTimeout( Type & obj, int timeoutMs=-1 );

protected:
    
    // The buffer itself
    std::deque<Type> queue_;

    // Reimplement this function for non-standard types.
    virtual void internalGet( Type & obj ) const ;

    // Reimplement this function for non-standard types.
    virtual void internalGet( Type & obj, unsigned int n ) const ;
    
    // Reimplement this function for non-standard types.
    virtual void internalPush( const Type & obj );

private:

    // buffer depth:
    //      positive numbers to specify finite depth,
    //      negative numbers for infinite depth (memory size),
    //      zero is undefined
    int depth_;

    // buffer type (see type definitions in BufferType enum)
    BufferType type_;

    // internal implementation of getWithTimeout( obj, -1 );
    void getWithInfiniteWait( Type & obj );
};


//////////////////////////////////////////////////////////////////////


template<class Type>
Buffer<Type>::Buffer( int depth, BufferType type )
    : depth_(depth),
      type_(type)
{
    purge();
}

template<class Type>
Buffer<Type>::~Buffer()
{
}

template<class Type>
void Buffer<Type>::configure( int depth, BufferType type )
{
    // all data is lost!
    purge();

    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    depth_ = depth;
    type_ = type;
}

template<class Type>
int 
Buffer<Type>::depth() const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    return depth_;
}

template<class Type>
BufferType 
Buffer<Type>::type() const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    return type_;
}

template<class Type>
void Buffer<Type>::purge()
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    queue_.resize(0);
}

template<class Type>
bool Buffer<Type>::isEmpty() const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    return queue_.empty();
}

template<class Type>
int Buffer<Type>::size() const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    return queue_.size();
}

template<class Type>
void Buffer<Type>::pop()
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    if ( queue_.empty() ) {
        return;
    }
    // must check for empty queue above, otherwise get seg fault!
    queue_.pop_front();
}

template<class Type>
void Buffer<Type>::get( Type &obj ) const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    if ( !queue_.empty() )
    {
        internalGet( obj );
    }
    else
    {
        throw gbxutilacfr::Exception( ERROR_INFO, "trying to read from an empty buffer." );
    }
}

template<class Type>
void Buffer<Type>::get( Type &obj, unsigned int n ) const
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    if ( queue_.empty() ){
        throw gbxutilacfr::Exception( ERROR_INFO, "trying to read from an empty buffer." );
    }
    else if( n >= queue_.size()){
        throw gbxutilacfr::Exception( ERROR_INFO, "index out of bounds while trying to read buffer." );
    }
    else{
        internalGet( obj ,n );
    }
}

template<class Type>
void Buffer<Type>::getAndPop( Type &obj )
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    if ( !queue_.empty() )
    {
        internalGet( obj );
    }
    else
    {
        throw gbxutilacfr::Exception( ERROR_INFO, "trying to read from an empty buffer." );
    }
    queue_.pop_front();
}

template<class Type>
int Buffer<Type>::getWithTimeout( Type &obj, int timeoutMs )
{
    // special case: infinite wait time
    if ( timeoutMs == -1 ) 
    {
        getWithInfiniteWait( obj );
        return 0;
    }

    // finite wait time
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

    // if already have data in the buffer, return it and get out
    if ( !queue_.empty() )
    {
        internalGet( obj );
        return 0;
    }

    // empty buffer: figure out when to wake up
    // notice that we are still holding the lock, so it's ok to call timedWait()
    if (  this->timedWait( IceUtil::Time::milliSeconds( timeoutMs ) ) )  
    {
        // someone woke us up, we are holding the lock again
        // check new data again (could be a spurious wakeup)
        if ( !queue_.empty() ) 
        {
            internalGet( obj );
            return 0;
        }
        else {
            // spurious wakup, don't wait again, just return
            return 1;
        }
    }
    else {
        // wait timedout, nobody woke us up
        return -1;
    }
}

template<class Type>
int Buffer<Type>::getAndPopWithTimeout( Type &obj, int timeoutMs )
{
    int ret = getWithTimeout( obj, timeoutMs );
    if ( ret==0 ) {
        pop();
    }
    return ret;
}

// internal utility function (waits for update infinitely)
template<class Type>
void Buffer<Type>::getWithInfiniteWait( Type &obj )
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    
    // check the condition before and after waiting to deal with spurious wakeups
    // (see Ice manual sec. 28.9.2)
    while ( queue_.empty() ) 
    {
        this->wait();
    }
    
    internalGet( obj );
}

// NOTE: see notes on efficient notification in Ice sec. 28.9.3
template<class Type>
void Buffer<Type>::push( const Type & obj )
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

    // buffer is not full, or buffer is configured to be of infinite size (at least for STL)
    if ( (int)queue_.size() < depth_ || depth_<0 )
    {
        internalPush( obj );
    }
    else if ( type_ == BufferTypeCircular )
    {
        // pop the oldest entry
        queue_.pop_front();
        // push the new enty
        internalPush( obj );
    }
    else // we have a full, non-circular buffer
    {
        // do nothing, the new object is lost
    }

    // wakeup someone who's waiting for an update
    this->notify();
}

template<class Type>
void Buffer<Type>::internalGet( Type & obj ) const
{
    obj = queue_.front();
}

template<class Type>
void Buffer<Type>::internalGet( Type & obj, unsigned int n ) const
{
    obj = queue_[n];
}

template<class Type>    
void Buffer<Type>::internalPush( const Type & obj )
{
    queue_.push_back( obj );
}

} // end namespace

#endif
