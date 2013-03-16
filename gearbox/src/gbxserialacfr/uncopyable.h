/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXSERIALACFR_UNCOPYABLE_H
#define GBXSERIALACFR_UNCOPYABLE_H

namespace gbxserialacfr {

//
// @brief Handy way to avoid unintended copies.  

// Inherit from this and the compiler will barf if you try to copy the derived class.
//
// @author Alex Brooks
//
class Uncopyable
{
public:
    Uncopyable() {}
private:
    Uncopyable(const Uncopyable&);
    void operator=(const Uncopyable&);
};

}

#endif
