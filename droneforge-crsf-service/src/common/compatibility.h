/* SPDX-License-Identifier: MIT
 * Copyright © Droneforge
 * 
 * Simple compatibility helpers - no complex C++20/C++23 features
 */
#pragma once

#include <cstdint>
#include <cstddef>

#if defined(_MSC_VER)
    #include <intrin.h>
#endif

// Simple byteswap for the specific types we need
namespace compat {

inline uint16_t byteswap16(uint16_t value) {
#if defined(_MSC_VER)
    return _byteswap_ushort(value);
#elif defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap16(value);
#else
    return ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);
#endif
}

inline uint32_t byteswap32(uint32_t value) {
#if defined(_MSC_VER)
    return _byteswap_ulong(value);
#elif defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap32(value);
#else
    return ((value >> 24) & 0x000000FF) |
           ((value >>  8) & 0x0000FF00) |
           ((value <<  8) & 0x00FF0000) |
           ((value << 24) & 0xFF000000);
#endif
}

// Simple span replacement
template<typename T>
class span {
private:
    T* data_;
    std::size_t size_;

public:
    constexpr span() : data_(nullptr), size_(0) {}
    constexpr span(T* data, std::size_t size) : data_(data), size_(size) {}
    
    constexpr T* data() const { return data_; }
    constexpr std::size_t size() const { return size_; }
    constexpr bool empty() const { return size_ == 0; }
    
    constexpr T& operator[](std::size_t idx) const { return data_[idx]; }
    
    constexpr T* begin() const { return data_; }
    constexpr T* end() const { return data_ + size_; }
    
    constexpr span subspan(std::size_t offset) const {
        return span(data_ + offset, size_ - offset);
    }
    
    constexpr span subspan(std::size_t offset, std::size_t count) const {
        return span(data_ + offset, count);
    }
};

} // namespace compat