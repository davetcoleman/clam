/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

/*dox
  @file 
  @brief Mathematical macros
  @ingroup orca_library_orcaiceutil 
 */

#ifndef GBXUTILACFR_MATH_DEFINITIONS_H
#define GBXUTILACFR_MATH_DEFINITIONS_H	

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

#include <assert.h>

/*****************************************************************************
 * INCLUDE THE RELEVANT MATHS LIBRARIES
 *****************************************************************************/

// MSVC compiler requires this symbol before exposing the (apparently) 
// non-standard symbols M_PI, etc...
#ifdef WIN32
#define _USE_MATH_DEFINES
#endif

#include <cmath>

/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
// M_PI is not defined after including cmath for the MS visual studio compiler?
#ifndef M_PI
//dox Defines number Pi
#define M_PI 3.14159265358979323846
#endif

#ifndef NAN
//dox Defines not-a-number
#define NAN (__builtin_nanf(""))
#endif

#ifndef INF
//dox Defines infinity
#define INF (__builtin_inff())
#endif

/*****************************************************************************
 * CONVERSION MACROS
 *****************************************************************************/
#ifndef DEG2RAD_RATIO
//dox Convertion factor from degrees to radians.
#define DEG2RAD_RATIO	(M_PI/180.0)
#endif

//dox Converts from degrees to radians.
#define DEG2RAD(deg)	((deg)*DEG2RAD_RATIO)
//dox Converts from radians to degrees.
#define RAD2DEG(rad) 	((rad)/DEG2RAD_RATIO)

//dox Normalises the angle [rad] to the range [-pi,pi)
//dox Don't return the normalised angle, because it's easy to make the
//dox mistake of doing: 'NORMALISE_ANGLE( myAngle )', ignoring the return value.
GBXUTILACFR_EXPORT inline void NORMALISE_ANGLE( double &theta )
{
    double multiplier;

    if (theta >= -M_PI && theta < M_PI)
        return;

    multiplier = std::floor(theta / (2*M_PI));
    theta -= multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    else if (theta < -M_PI)
        theta += 2*M_PI;
}

//dox Normalises the angle [rad] to the range [-pi,pi)
//dox Don't return the normalised angle, because it's easy to make the
//dox mistake of doing: 'NORMALISE_ANGLE( myAngle )', ignoring the return value.
GBXUTILACFR_EXPORT inline void NORMALISE_ANGLE( float &theta )
{
    double thDouble = theta;
    NORMALISE_ANGLE( thDouble );
    theta = (float)thDouble;
}

/*****************************************************************************
 * MATH MACROS
 *****************************************************************************/
// for compatability retain this old one
//#ifndef ABS
//#define ABS(x)           (std::abs(x))
//#endif
#ifndef MIN
//dox Minimum of two numbers
#define MIN(x, y)        (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
//dox Maximum of two numbers
#define MAX(x, y)        (((x) > (y)) ? (x) : (y))
#endif
#ifndef APPLY_LIMITS
//dox Limits x to an interval between min_x and max_x.
//dox x must be a variable which can be assigned a value;
//dox Compares using less_than and greater_than.
#define APPLY_LIMITS(max_x, x, min_x) \
        if((x)>(max_x)) x=(max_x); if((x)<(min_x)) x=(min_x);
#endif
#ifndef NORM2
//dox Norm of a 2D vector
#define NORM2(x, y)      (sqrt((x)*(x)+(y)*(y)))
#endif
#ifndef NORM3
//dox Norm of a 3D vector
#define NORM3(x, y, z)   (sqrt((x)*(x)+(y)*(y)+(z)*(z)))
#endif
#ifndef ROUND
//dox Rounds double or float to the nearest integer.
#define ROUND(x)         ((int)(x+0.5))
#endif
#ifndef ROUND_TO
//dox Rounds @c n to the nearest whole number of multiples of @c d.
//dox For example, ROUND_TO(8,5) and ROUND_TO(12,5) will both result in 10.
#define ROUND_TO(n,d)    (d*rint(n/d))
#endif
#ifndef SIGN
//dox Sign of a number.
#define SIGN(A)          ((A)<0?(-1):(1))
#endif
#ifndef COS_LAW
//dox Law of cosines.
#define COS_LAW(side1, side2, theta) \
        (sqrt(SQR(side1)+SQR(side2)-2.0*(side1)*(side2)*cos(theta)))
#endif
#ifndef INV_COS_LAW
//dox Inverse law of cosines..
#define INV_COS_LAW(oppSide, side1, side2) \
        (acos((SQR(side1)+SQR(side2)-SQR(oppSide))/(2.0*(side1)*(side2))))
#endif

// isinf not defined on all systems (eg Solaris)
#if defined (__SVR4) && defined (__sun)
#define	isfinite(x) \
  __extension__ ({ __typeof (x) __x_f = (x); \
		   __builtin_expect(!isnan(__x_f - __x_f), 1); })

#define	isinf(x) \
  __extension__ ({ __typeof (x) __x_i = (x); \
		   __builtin_expect(!isnan(__x_i) && !isfinite(__x_i), 0); })
#endif


/*****************************************************************************
 * COMPARISON MACROS
 *****************************************************************************/
#ifndef NEAR
//dox Check that two numbers are sufficiently close.
//dox  Compares using less_than and greater_than.
#define NEAR(x,y,epsilon) (((x) > (y)-(epsilon)) && ((x) < (y)+(epsilon))) 
#endif

//dox Modifies x to lie within [x_min,x_max]
//dox
template<typename T>
GBXUTILACFR_EXPORT void
CLIP_TO_LIMITS( const T &min_x, T &x, const T &max_x )
{
    assert( min_x <= max_x );
    if ( x > max_x )
        x = max_x;
    else if ( x < min_x )
        x = min_x;
}
     
#endif
