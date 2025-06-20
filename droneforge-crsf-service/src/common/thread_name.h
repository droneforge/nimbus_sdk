/* SPDX-License-Identifier: MIT
 * Copyright © Mark Collins
 */
#pragma once

#include <string>

#ifdef _WIN32
    #include <windows.h>
    #include <processthreadsapi.h>
#elif defined(__linux__)
    #include <pthread.h>
#endif

/**
 * @brief Set the name of the calling thread.
 * @note This may silently fail in some cases, e.g. the name is too long.
 */
inline void SetThreadName(const std::string& name) {
#ifdef _WIN32

    std::wstring wName{name.begin(), name.end()};
    SetThreadDescription(GetCurrentThread(), wName.c_str());

#elif defined(__linux__)
    pthread_setname_np(pthread_self(), name.c_str());
#endif
}