/* SPDX-License-Identifier: MIT
 * Copyright © Mark Collins
 */
#pragma once

#include <fmt/format.h>
#include <stdexcept>

/**
 * @brief A runtime error with a formatted message.
 */
struct Exception : std::runtime_error {
    template <typename... Args>
    constexpr explicit Exception(fmt::format_string<Args...> str, Args&&... args)
        : std::runtime_error{fmt::format(str, std::forward<Args>(args)...)} {}
};

/**
 * @brief Checks a condition and terminates with an error message, if it is true. Designed as an alternative to exceptions in noexcept functions.
 */
template <typename... Args>
constexpr void TerminateIf(bool condition, fmt::format_string<Args...> str, Args&&... args) {
    if (condition) {
        fmt::print(stderr, "Terminating: {}\n", fmt::format(str, std::forward<Args>(args)...));
        std::terminate();
    }
}