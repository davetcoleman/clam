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

#ifndef UTILS_H__
#define UTILS_H__

#include <flexiport/port.h>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>

#include <iostream>

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

/** @ingroup gbx_library_hokuyo_aist
@{
*/

namespace hokuyo_aist
{

#ifndef M_PI
    double const M_PI = 3.14159265358979323846;
#endif
// Convert radians to degrees
#ifndef RTOD
    inline double RTOD(double rad)
    {
        return rad * 180.0 / M_PI;
    }
#endif
// Convert degrees to radians
#ifndef DTOR
    inline double DTOR(double deg)
    {
        return deg * M_PI / 180.0;
    }
#endif


/// Find the median value of a std::vector.
template<typename T>
inline T median(std::vector<T>& v)
{
    typename std::vector<T>::iterator first(v.begin());
    typename std::vector<T>::iterator median(first + (v.end() - first) / 2);
    std::nth_element(first, median, v.end());
    return *median;
}

} // namespace hokuyo_aist

/** @} */

#endif // UTILS_H__

