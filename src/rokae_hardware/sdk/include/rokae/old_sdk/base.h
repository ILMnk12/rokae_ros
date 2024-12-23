/**
 * @file base.h
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_BASE_H
#define ROKAEAPI_BASE_H

#include <memory>
#include <functional>
#include <system_error>
#include <set>

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

// *********************           Macros          ************************
// *********************           宏定义           ************************
#if defined(ROKAE_DLL_BUILD)
#if defined(_MSC_VER)
    #ifdef ROKAE_LIBRARY
        #define ROKAE_EXPORT __declspec(dllexport)
    #else
        #define ROKAE_EXPORT __declspec(dllimport)
    #endif
#else // if defined(_MSC_VER)
    #define ROKAE_EXPORT __attribute__((visibility("default")))
#endif  // if defined(_MSC_VER)
#endif // if defined(ROKAE_DLL_BUILD)

#if !defined(ROKAE_EXPORT)
#define ROKAE_EXPORT
#endif

typedef std::error_code error_code;

#ifndef ROKAE_DECLARE_IMPL
#define ROKAE_DECLARE_IMPL \
protected: \
    class Impl; \
    Impl* impl_;
#endif

#ifndef ROKAE_DECLARE_IMPLD
#define ROKAE_DECLARE_IMPLD \
protected: \
    class Impld; \
    Impld* impld_;
#endif

namespace rokae {
/// @cond DO_NOT_DOCUMENT
 template<class T> struct Base {};
/// @endcond
}

#endif //ROKAEAPI_BASE_H
