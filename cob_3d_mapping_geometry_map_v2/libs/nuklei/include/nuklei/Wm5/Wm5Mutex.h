// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)

#ifndef NUKLEI_WMFMUTEX_H
#define NUKLEI_WMFMUTEX_H

#include "Wm5CoreLIB.h"
#include "Wm5MutexType.h"

namespace nuklei_wmf
{

class NUKLEI_WMF_CORE_ITEM Mutex
{
public:
    // Construction and destruction.
    Mutex ();
    ~Mutex ();

    // Markers for critical sections.
    void Enter ();
    void Leave ();

private:
    MutexType mMutex;
};

}

#endif
