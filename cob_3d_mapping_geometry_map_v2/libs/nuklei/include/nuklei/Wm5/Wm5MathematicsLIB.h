// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.3 (2010/10/01)

#ifndef NUKLEI_WMFMATHEMATICSLIB_H
#define NUKLEI_WMFMATHEMATICSLIB_H

#include "Wm5CoreLIB.h"

// Begin Microsoft Windows DLL support.
#if defined(NUKLEI_WMF_MATHEMATICS_DLL_EXPORT)
    // For the DLL library.
    #define NUKLEI_WMF_MATHEMATICS_ITEM __declspec(dllexport)
#elif defined(NUKLEI_WMF_MATHEMATICS_DLL_IMPORT)
    // For a client of the DLL library.
    #define NUKLEI_WMF_MATHEMATICS_ITEM __declspec(dllimport)
#else
    // For the static library and for Apple/Linux.
    #define NUKLEI_WMF_MATHEMATICS_ITEM
#endif
// End Microsoft Windows DLL support.

// Enable this define if you want the Rational class to assert when the
// constructor is passed a floating-point infinity or NaN.
//#define NUKLEI_WMF_ASSERT_ON_RATIONAL_CONVERT_NAN

// Enable this define if you want Vector2<Real>::GetBarycentrics to assert
// when the input triangle is degenerate.
//#define NUKLEI_WMF_ASSERT_ON_BARYCENTRIC2_DEGENERATE

// Enable this define if you want Vector3<Real>::GetBarycentrics to assert
// when the input tetrahedron is degenerate.
//#define NUKLEI_WMF_ASSERT_ON_BARYCENTRIC3_DEGENERATE

#endif
