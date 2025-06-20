/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright © ExpressLRS Developers
 * Copyright © Mark Collins
 * Heavily derivative of ExpressLRS, see https://github.com/ExpressLRS/ExpressLRS/tree/f3cc5e66796c93b5eaa90a3d1d519f0d0fa0e4ea/src/lib/CrsfProtocol
 */

#pragma once

#include "crc.h"
#include "../common/types.h"
#include "../common/compatibility.h"
#include <array>

namespace crsf {
constexpr u8 CrcPolynomial{0xd5};
constexpr PolynomialCrc CrsfCrc{CrcPolynomial};

constexpr u8 SyncByte{0xC8};

constexpr size_t FrameSizeNotCountedBytes{2};
constexpr size_t PayloadSizeMax{62};
constexpr size_t FrameTypeSize{1}, FrameCrcSize{1};
constexpr size_t FrameSizeMax{FrameSizeNotCountedBytes + PayloadSizeMax + FrameCrcSize};

enum class FrameType : u8 {
    Gps = 0x02,
    Vario = 0x07,
    BatterySensor = 0x08,
    BaroAltitude = 0x09,
    LinkStatistics = 0x14,
    OpenTxSync = 0x10,
    RadioId = 0x3A,
    RcChannelsPacked = 0x16,
    Attitude = 0x1E,
    FlightMode = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    DevicePing = 0x28,
    DeviceInfo = 0x29,
    ParameterSettingsEntry = 0x2B,
    ParameterRead = 0x2C,
    ParameterWrite = 0x2D,
    ElrsStatus = 0x2E, // ELRS good/bad packet count and status flags
    Command = 0x32,
    // KISS frames
    KissRequest = 0x78,
    KissResponse = 0x79,
    // MSP commands
    MspRequest = 0x7A, // Response request using msp sequence as command
    MspResponse = 0x7B, // Reply with 58 byte chunked binary
    MspWrite = 0x7C, // Write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    ArdupilotResp = 0x80,
};

enum class CommandType : u8 {
    Rx = 0x10
};

enum class SubCommandType : u8 {
    RxBind = 0x01,
    ModelSelectId = 0x05
};

enum class Address : u8 {
    Broadcast = 0x00,
    Usb = 0x10,
    TbsCorePnpPro = 0x80,
    Reserved1 = 0x8A,
    CurrentSensor = 0xC0,
    Gps = 0xC2,
    TbsBlackbox = 0xC4,
    FlightController = 0xC8,
    Reserved2 = 0xCA,
    RaceTag = 0xCC,
    RadioTransmitter = 0xEA,
    CrsfReceiver = 0xEC,
    CrsfTransmitter = 0xEE,
    ElrsLua = 0xEF
};

// Common device addresses to ping during initialization
constexpr std::array<Address, 5> DeviceAddressList{{
    Address::FlightController,
    Address::CrsfReceiver,
    Address::CrsfTransmitter,
    Address::ElrsLua,
    Address::RadioTransmitter
}};

#pragma pack(push, 1)
struct Header {
    Address deviceAddress;
    u8 frameSize; //!< Size of the frame, for all bytes after the frameSize byte, including the CRC.
    FrameType type;
};

constexpr size_t HeaderSize{sizeof(Header)};

struct ExtHeader {
    Address deviceAddr;
    u8 frameSize;
    FrameType type;
    Address destinationAddress;
    Address originAddress;
};

constexpr size_t ExtSize{sizeof(ExtHeader)};

enum class ParameterType : u8 {
    Uint8 = 0,
    Int8 = 1,
    Uint16 = 2,
    Int16 = 3,
    Uint32 = 4,
    Int32 = 5,
    Uint64 = 6,
    Int64 = 7,
    Float = 8,
    TextSelection = 9,
    String = 10,
    Folder = 11,
    Info = 12,
    Command = 13,
    Vtx = 15,
    OutOfRange = 127,
};

constexpr u8 ParameterHidden{0x80};
constexpr u8 ParameterElrsHidden{0x40};
constexpr u8 ParameterTypeMask{static_cast<u8>(~(ParameterHidden | ParameterElrsHidden))};

enum class CommandStep : u8 {
    Idle = 0,
    Click = 1,
    Executing = 2,
    AskConfirm = 3,
    Confirmed = 4,
    Cancel = 5,
    Query = 6,
};

constexpr const char* CommandStepToString(CommandStep step) {
    switch (step) {
        case CommandStep::Idle:
            return "Idle";
        case CommandStep::Click:
            return "Clicked";
        case CommandStep::Executing:
            return "Executing";
        case CommandStep::AskConfirm:
            return "Needs Confirmation";
        case CommandStep::Confirmed:
            return "Confirmed";
        case CommandStep::Cancel:
            return "Cancel";
        case CommandStep::Query:
            return "Query";
        default:
            return "Unknown";
    }
}

struct RcChannelsData {
    static constexpr size_t ChannelCount{16};
    static constexpr size_t BitsPerChannel{11};
    static constexpr size_t PackedSize{((BitsPerChannel * ChannelCount) + 7) / 8};
    static constexpr u16 BitMask{(1 << BitsPerChannel) - 1};

    std::array<u8, PackedSize> data;

    constexpr bool operator==(const RcChannelsData& other) const {
        return data == other.data;
    }

    constexpr bool operator!=(const RcChannelsData& other) const {
        return data != other.data;
    }

    constexpr u16 get(u8 i) const {
        size_t byteIndex{(i * BitsPerChannel) / 8};
        size_t bitOffset{(i * BitsPerChannel) % 8};
        u16 value{static_cast<u16>((data[byteIndex] >> bitOffset) | (data[byteIndex + 1] << (8 - bitOffset)))};
        if (bitOffset > 5)
            value |= (data[byteIndex + 2] << (16 - bitOffset));
        return value & BitMask;
    }

    constexpr void set(u8 i, u16 val) {
        size_t byteIndex{(i * BitsPerChannel) / 8};
        size_t bitOffset{(i * BitsPerChannel) % 8};
        data[byteIndex] = (data[byteIndex] & ~(BitMask << bitOffset)) | ((val << bitOffset) & 0xFF);
        data[byteIndex + 1] = (data[byteIndex + 1] & ~(BitMask >> (8 - bitOffset))) | ((val >> (8 - bitOffset)) & 0xFF);
        if (bitOffset > 5)
            data[byteIndex + 2] = (data[byteIndex + 2] & (0xFF << (16 - bitOffset))) | ((val >> (16 - bitOffset)) & 0xFF);
    }
};

struct DeviceInformationData {
    u32 serialNo;
    u32 hardwareVersion;
    u32 softwareVersion;
    u8 ParametersCount; //!< Number of parameters in the device, these can be queried using ParameterRead frames.
    u8 ParametersVersion;
    // A null-terminated string containing the device name follows.
};

namespace msp {
    constexpr size_t RequestPayloadSize{8};
    constexpr size_t ResponsePayloadSize{58};
    constexpr size_t MaxPayloadSize{(RequestPayloadSize > ResponsePayloadSize ? RequestPayloadSize : ResponsePayloadSize)};

    struct VtxConfigPacket {
        u8 vtxType;
        u8 band;
        u8 channel;
        u8 power;
        u8 pitmode;
        u16 freq;
        u8 deviceIsReady;
        u8 lowPowerDisarm;
        u16 pitModeFreq;
        u8 vtxTableAvailable;
        u8 bands;
        u8 channels;
        u8 powerLevels;
    };

    struct VtxPowerLevelPacket {
        u8 powerLevel;
        u16 powerValue;
        u8 powerLabelLength;
        u8 label[3];
    };

    struct VtxBandPacket {
        u8 band;
        u8 bandNameLength;
        u8 bandName[8];
        u8 bandLetter;
        u8 isFactoryBand;
        u8 channels;
        u16 channel[8];
    };
}

struct BatterySensorData {
    u16 voltage; // mv * 100, big endian.
    u16 current; // ma * 100, big endian.
    u32 capacityRemainingPacked; // First 24 bits capacity in mah, last 8 bits remaining capacity in %, little endian.

    /*
     * @return Battery voltage in volts.
     */
    inline float Voltage() const {
        return compat::byteswap16(voltage) / 10.0f;
    }

    /*
     * @return Current draw in amps.
     */
    inline float Current() const {
        return compat::byteswap16(current) / 100.0f;
    }

    /*
     * @return Battery capacity in mAh.
     */
    constexpr float Capacity() const {
        return (capacityRemainingPacked & 0xFFFFFF) / 1000.0f;
    }

    /*
     * @return Remaining battery capacity in %.
     */
    constexpr u8 RemainingCapacity() const {
        return capacityRemainingPacked >> 24;
    }
};

struct BaroVarioData {
    u16 altitude; //!< Altitude in decimeters + 10000dm, or Altitude in meters if high bit is set, big endian.
    i16 verticalSpeed; //!< Vertical speed in cm/s, big endian.

    /*
     * @return Altitude in meters.
     */
    inline float Altitude() const {
        u16 alt{compat::byteswap16(altitude)};
        if (alt & 0x8000)
            return static_cast<float>(alt & 0x7FFF);
        return static_cast<float>(alt - 10000) / 10.0f;
    }

    /*
     * @return Vertical speed in m/s.
     */
    inline i16 VerticalSpeed() const {
        return compat::byteswap16(verticalSpeed);
    }
};

struct VarioData {
    i16 verticalSpeed; //!< Vertical speed in cm/s, big endian.

    /*
     * @return Vertical speed in m/s.
     */
    inline float VerticalSpeed() const {
        return compat::byteswap16(verticalSpeed) / 100.0f;
    }
};

struct GpsData {
    i32 latitude; //!< Degree / 10`000`000.
    i32 longitude; //!< Degree / 10`000`000.
    u16 groundSpeed; //!< km/h / 10.
    u16 gpsHeading; //!< Degree / 100.
    u16 altitude; //!< Meters, ­1000m offset.
    u8 satellitesInUse;

    /*
     * @return Latitude in degrees.
     */
    constexpr double Latitude() const {
        return static_cast<float>(latitude) / 10000000.0f;
    }

    /*
     * @return Longitude in degrees.
     */
    constexpr double Longitude() const {
        return static_cast<float>(longitude) / 10000000.0f;
    }

    /*
     * @return Ground speed in km/h.
     */
    constexpr float GroundSpeed() const {
        return static_cast<float>(groundSpeed) / 10.0f;
    }

    /*
     * @return GPS heading in degrees.
     */
    constexpr float GpsHeading() const {
        return static_cast<float>(gpsHeading) / 100.0f;
    }

    /*
     * @return Altitude in meters, relative to the ground.
     */
    constexpr float Altitude() const {
        return static_cast<float>(altitude) + 1000.0f;
    }
};

struct AttitudeData {
    int16_t pitch;  // Pitch angle in degrees * 10
    int16_t roll;   // Roll angle in degrees * 10
    int16_t yaw;    // Yaw angle in degrees * 10

    constexpr float Pitch() const { return pitch / 10.0f; }
    constexpr float Roll() const { return roll / 10.0f; }
    constexpr float Yaw() const { return yaw / 10.0f; }
};

struct FlightModeData {
    std::array<char, 16> flightMode;
};

struct LinkStatisticsData {
    static constexpr float MaxRssi{300.f}; //!< An arbitrary maximum RSSI value, this is inverted.
    static constexpr float MaxSnrDb{100.f}; //!< Maximum SNR in dB, this value is centered around 0 going in positive and negative directions.

    u8 uplinkRssi1Inverse;
    u8 uplinkRssi2Inverse;
    u8 uplinkLinkQuality; //!< The link quality in %.
    i8 uplinkSnr;
    u8 activeAntenna; //!< 0 = Antenna 1, 1 = Antenna 2.
    u8 rfModeIndex;
    u8 uplinkTxPowerIndex;
    u8 downlinkRssiInverse;
    u8 downlinkLinkQuality;
    i8 downlinkSnr;

    constexpr float UplinkRssi1() const {
        return static_cast<float>(uplinkRssi1Inverse) * -1.0f;
    }

    constexpr float UplinkRssi1Norm() const {
        return (MaxRssi + UplinkRssi1()) / MaxRssi;
    }

    constexpr float UplinkRssi2() const {
        return static_cast<float>(uplinkRssi2Inverse) * -1.0f;
    }

    constexpr float UplinkRssi2Norm() const {
        return (MaxRssi + UplinkRssi2()) / MaxRssi;
    }

    constexpr float UplinkLinkQualityNorm() const {
        return static_cast<float>(uplinkLinkQuality) / 100.0f;
    }

    constexpr float UplinkSnrNorm() const {
        return (static_cast<float>(uplinkSnr) + MaxSnrDb) / (MaxSnrDb * 2);
    }

    constexpr float DownlinkRssi() const {
        return static_cast<float>(downlinkRssiInverse) * -1.0f;
    }

    constexpr float DownlinkRssiNorm() const {
        return (MaxRssi + DownlinkRssi()) / MaxRssi;
    }

    constexpr float DownlinkLinkQualityNorm() const {
        return static_cast<float>(downlinkLinkQuality) / 100.0f;
    }

    constexpr float DownlinkSnrNorm() const {
        return (static_cast<float>(downlinkSnr) + MaxSnrDb) / (MaxSnrDb * 2);
    }

    constexpr const char* RfMode() const {
        constexpr std::array<const char*, 20> ExpressLrsRfModes{
            "LORA 4Hz",
            "LORA 25Hz",
            "LORA 50Hz",
            "LORA 100Hz",
            "LORA 100Hz 8CH",
            "LORA 150Hz",
            "LORA 200Hz",
            "LORA 250Hz",
            "LORA 333Hz 8CH",
            "LORA 500Hz",
            "DVDA 250Hz (FLRC)",
            "DVDA 500Hz (FLRC)",
            "FLRC 500Hz",
            "FLRC 1000Hz",
            "DVDA 50Hz",
            "LORA 200Hz 8CH",
            "FSK 2G4 DVDA 500Hz",
            "FSK 2G4 1000Hz",
            "FSK 900 1000Hz",
            "FSK 900 1000Hz 8CH"};
        if (rfModeIndex > ExpressLrsRfModes.size())
            return "Unknown";
        return ExpressLrsRfModes[rfModeIndex];
    }

    constexpr int UplinkTxPower() const {
        constexpr std::array<int, 9> TxPowers{0, 10, 25, 100, 500, 1000, 2000, 250, 50};
        if (uplinkTxPowerIndex > TxPowers.size())
            return -1;
        return TxPowers[uplinkTxPowerIndex];
    }
};

#pragma pack(pop)

} // namespace crsf
