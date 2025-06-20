/* SPDX-License-Identifier: MIT
 * Copyright © Mark Collins
 */
#pragma once

#include <array>
#include <stdexcept>
#include "compatibility.h"

/**
 * @brief A minimal implementation of boost's static_vector.
 */
template <typename T, size_t MaxSize>
class StaticVector {
  private:
    std::array<T, MaxSize> buffer{};

  public:
    size_t size{};

    constexpr StaticVector() = default;

    template <typename BeginIt, typename EndIt>
    constexpr StaticVector(BeginIt begin, EndIt end)
        : size{static_cast<size_t>(end - begin)} {
        if (size > MaxSize)
            throw std::out_of_range("StaticVector size exceeds MaxSize");
        std::copy(begin, end, buffer.begin());
    }

    constexpr bool operator==(const StaticVector& other) const {
        if (size != other.size)
            return false;
        return std::equal(buffer.begin(), buffer.begin() + size, other.buffer.begin());
    }

    constexpr bool operator!=(const StaticVector& other) const {
        return !(*this == other);
    }

    constexpr T* data() {
        return buffer.data();
    }

    constexpr void resize(size_t newSize) {
        if (newSize > MaxSize)
            throw std::out_of_range("StaticVector size exceeds MaxSize");
        size = newSize;
    }

    constexpr T* data_resize(size_t newSize) {
        resize(newSize);
        return data();
    }

    constexpr T* begin() {
        return buffer.data();
    }

    constexpr T* end() {
        return buffer.data() + size;
    }

    constexpr T& operator[](size_t index) {
        return buffer[index];
    }

    constexpr T& at(size_t index) {
        return buffer.at(index);
    }
    
    // Conversion operator to compat::span
    constexpr operator compat::span<T>() const {
        return compat::span<T>(const_cast<T*>(buffer.data()), size);
    }
};
