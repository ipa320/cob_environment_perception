// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.3 (2010/10/01)

#ifndef NUKLEI_WMFCORELIB_H
#define NUKLEI_WMFCORELIB_H

//----------------------------------------------------------------------------
// Platform-specific information.  The defines to control which platform is
// included are listed below.  Add others as needed.
//
// _WIN32 or WIN32  :  Microsoft Windows
// __MINGW32__      :  Minimalist GNU for Windows
// __CYGWIN__       :  Cygwin
// __APPLE__        :  Macintosh OS X
// __LINUX__        :  Linux
//----------------------------------------------------------------------------
// Microsoft Windows platform
//----------------------------------------------------------------------------
#if defined(_WIN32) || defined(WIN32)

#ifndef _WIN32
#define _WIN32
#endif

#ifndef WIN32
#define WIN32
#endif

#define NUKLEI_WMF_LITTLE_ENDIAN

#if defined(_MSC_VER)
// MSVC6 is version 12.00, MSVC7.0 is version 13.00, MSVC7.1 is version 13.10,
// MSVC8.0 is version 14.00, and MSVC9.0 is version 15.00.  Currently, only
// version 9.0 or later are supported.
#if _MSC_VER < 1400
#error MSVC 8.0 or later is required
#elif _MSC_VER < 1500
#define NUKLEI_WMF_USING_VC80
#else
#define NUKLEI_WMF_USING_VC90
#endif

// Disable the Microsoft warnings about not using the secure functions.
#pragma warning(disable : 4996)

// The use of NUKLEI_WMF_<libname>_ITEM to export an entire class generates warnings
// when member data and functions involving templates or inlines occur.  To
// avoid the warning, NUKLEI_WMF_<libname>_ITEM can be applied only to those items
// that really need to be exported.
#pragma warning(disable : 4251) 

// Support for standard integer types.  This is only a small part of what
// stdint.h provides on Unix platforms.
#include <climits>

typedef __int8              int8_t;
typedef __int16             int16_t;
typedef __int32             int32_t;
typedef __int64             int64_t;
typedef unsigned __int8     uint8_t;
typedef unsigned __int16    uint16_t;
typedef unsigned __int32    uint32_t;
typedef unsigned __int64    uint64_t;

#define INT8_MIN            _I8_MIN
#define INT8_MAX            _I8_MAX
#define INT16_MIN           _I16_MIN
#define INT16_MAX           _I16_MAX
#define INT32_MIN           _I32_MIN
#define INT32_MAX           _I32_MAX
#define INT64_MIN           _I64_MIN
#define INT64_MAX           _I64_MAX
#define UINT8_MAX           _UI8_MAX
#define UINT16_MAX          _UI16_MAX
#define UINT32_MAX          _UI32_MAX
#define UINT64_MAX          _UI64_MAX
#endif

#endif
//----------------------------------------------------------------------------
// Macintosh OS X platform
//----------------------------------------------------------------------------
#if defined(__APPLE__)

#if defined(__BIG_ENDIAN__)
#define NUKLEI_WMF_BIG_ENDIAN
#else
#define NUKLEI_WMF_LITTLE_ENDIAN
#endif

#endif
//----------------------------------------------------------------------------
// PC Linux platform
//----------------------------------------------------------------------------
#if defined(__LINUX__) || defined(__linux__)

// Support for standard integer types.
#include <inttypes.h>

#define NUKLEI_WMF_LITTLE_ENDIAN

#endif
//----------------------------------------------------------------------------

// Begin Microsoft Windows DLL support.
#if defined(NUKLEI_WMF_CORE_DLL_EXPORT)
    // For the DLL library.
    #define NUKLEI_WMF_CORE_ITEM __declspec(dllexport)
#elif defined(NUKLEI_WMF_CORE_DLL_IMPORT)
    // For a client of the DLL library.
    #define NUKLEI_WMF_CORE_ITEM __declspec(dllimport)
#else
    // For the static library and for Apple/Linux.
    #define NUKLEI_WMF_CORE_ITEM
#endif
// End Microsoft Windows DLL support.

// Common standard library headers.
#include <cassert>
#include <cctype>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <fstream>

// Common STL headers.
#include <algorithm>
#include <deque>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <utility>
#include <vector>

// User-defined keywords for syntax highlighting of special class sections.
#define public_internal public
#define protected_internal protected
#define private_internal private

// Avoid warnings about unused variables.  This is designed for variables
// that are exposed in debug configurations but are hidden in release
// configurations.
#define NUKLEI_WMF_UNUSED(variable) (void)variable

// Macros to enable or disable various Assert subsystems.  TODO:  Currently,
// the extended assert system is implemented only for Microsoft Visual Studio.
#ifdef _DEBUG
    #if defined(WIN32) && defined(_MSC_VER)
        #define NUKLEI_WMF_USE_ASSERT
        #ifdef NUKLEI_WMF_USE_ASSERT
            #define NUKLEI_WMF_USE_ASSERT_WRITE_TO_OUTPUT_WINDOW
            #define NUKLEI_WMF_USE_ASSERT_WRITE_TO_MESSAGE_BOX
        #endif
    #endif
    //#define NUKLEI_WMF_USE_ASSERT_LOG_TO_FILE
#endif

// Macros for memory management.
#ifdef _DEBUG
    #define NUKLEI_WMF_USE_MEMORY
    #ifdef NUKLEI_WMF_USE_MEMORY
        // Enable an assertion when a allocation occurs before
        // Memory::Initialize was called or when a deallocation occurs after
        // Memory::Terminate was called.
        #define NUKLEI_WMF_USE_MEMORY_ASSERT_ON_PREINIT_POSTTERM_OPERATIONS

        // When NUKLEI_WMF_USE_MEMORY is enabled, the Memory functions delete0,
        // delete1, delete2, and delete3 will fail when the incoming pointer
        // was not allocated by the Wild Magic memory manager.  Enabling the
        // following flag tells Wild Magic's memory manager to attempt
        // 'delete' or 'delete[]' under the assumption that the memory was
        // allocated by 'new' or 'new[]'.
        #define NUKLEI_WMF_USE_MEMORY_ALLOW_DELETE_ON_FAILED_MAP_LOOKUP
    #endif
#endif

// Flags for FileIO and BufferIO.
#ifdef _DEBUG
    #define NUKLEI_WMF_FILEIO_VALIDATE_OPERATION
    #define NUKLEI_WMF_BUFFERIO_VALIDATE_OPERATION
#endif

#endif
