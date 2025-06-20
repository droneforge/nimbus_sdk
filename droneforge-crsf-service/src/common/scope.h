/* SPDX-License-Identifier: MIT
 * Copyright © Mark Collins
 */
#pragma once

template <typename T>
struct ScopeExit {
    T lambda;
    ScopeExit(T lambda)
        : lambda(lambda) {}

    ~ScopeExit() {
        lambda();
    }
};

#define SCOPE_EXIT(code) ScopeExit _scopeExit ## __LINE__{[&] { code; }}