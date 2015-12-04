// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)

#ifndef NUKLEI_WMFASSERT_H
#define NUKLEI_WMFASSERT_H

#include "Wm5CoreLIB.h"

#ifdef NUKLEI_WMF_USE_ASSERT
//----------------------------------------------------------------------------
// Use NUKLEI_WMF asserts with file/line tracking.
//----------------------------------------------------------------------------
namespace nuklei_wmf
{

class NUKLEI_WMF_CORE_ITEM Assert
{
public:
    // Construction and destruction.
    Assert (bool condition, const char* file, int line, const char* format,
        ...);

    ~Assert ();

private:
    enum { MAX_MESSAGE_BYTES = 1024 };
    static const char* msDebugPrompt;
    static const size_t msDebugPromptLength;
    static const char* msMessagePrefix;

#ifdef NUKLEI_WMF_USE_ASSERT_WRITE_TO_MESSAGE_BOX
    static const char* msMessageBoxTitle;
#endif
};

}

#define assertion(condition, format, ...) \
    nuklei_wmf::Assert(condition, __FILE__, __LINE__, format, __VA_ARGS__)
//----------------------------------------------------------------------------
#else
//----------------------------------------------------------------------------
// Use standard asserts.
//----------------------------------------------------------------------------
#define assertion(condition, format, ...) assert(condition)
//----------------------------------------------------------------------------
#endif

#endif
