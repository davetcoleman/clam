/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <gbxutilacfr/mathdefs.h>
#include <iomanip>
#include <assert.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

using namespace std;

#define EPS 1e-8;

template<typename T>
void
testNormalise()
{
    T angle;

    angle = (T)(-M_PI);
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );

    angle = -(T)M_PI + (T)EPS;
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );

    angle = -(T)M_PI - (T)EPS;
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );

    angle = (T)M_PI;
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );

    angle = (T)M_PI + (T)EPS;
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );

    angle = (T)M_PI - (T)EPS;
    NORMALISE_ANGLE(angle);
    assert( angle >= -M_PI && angle < M_PI );
}

int main()
{
    testNormalise<float>();
    testNormalise<double>();

    cout << "Test PASSED" << endl;
}
