/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright © ExpressLRS Developers
 * Copyright © Mark Collins
 * Heavily derivative of ExpressLRS, see https://github.com/ExpressLRS/ExpressLRS/tree/f3cc5e66796c93b5eaa90a3d1d519f0d0fa0e4ea/src/lib/CRC
 */

#pragma once

#include <array>
#include "../common/types.h"

namespace crsf {
/*
 * @brief A CRC8 implementation using a lookup table computed at compile time.
 */
class PolynomialCrc {
  private:
    std::array<u8, UINT8_MAX + 1> table{}; //!< A lookup table containing the mapping for every byte value.

  public:
    constexpr PolynomialCrc(u8 polynomial) : table{} {
        constexpr u8 BitsInByte{8};

        // Compute CRC lookup table
        for (size_t i = 0; i < table.size(); i++) {
            size_t crc{i};
            for (size_t j = 0; j < BitsInByte; j++)
                crc = (crc << 1) ^ ((crc & (1 << (BitsInByte - 1))) ? polynomial : 0);
            table[i] = static_cast<u8>(crc & UINT8_MAX);
        }
    }

    constexpr u8 Calculate(u8 data) const {
        return table[data];
    }

    constexpr u8 Calculate(const u8* data, uint16_t length, u8 crc = 0) const {
        while (length--)
            crc = table[crc ^ *data++];
        return crc;
    }
};
} 