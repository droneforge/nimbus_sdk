/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#pragma once

#include <optional>
#include <string>
#include <variant>

#include "../common/error.h"
#include "../common/static_vector.h"
#include "../common/compatibility.h"

#include "protocol.h"

namespace crsf {
constexpr Address DefaultOriginAddress{Address::RadioTransmitter}; //!< Default origin address is the radio transmitter, since this is designed to emulate it.
constexpr Address DefaultDeviceAddress{Address::CrsfTransmitter}; //!< Default device address is the CRSF TX, since the radio is going to be attached to it.

using FrameData = StaticVector<u8, FrameSizeMax>;

/**
 * @brief The base class for all CRSF frames.
 * @note Any frame that needs to be recieved must have a constructor that takes a compat::span<u8> as an argument.
 * @note Any frame that needs to be sent must implement the WritePayload virtual method.
 */
struct Frame {
    Address deviceAddress;
    FrameType type;

    constexpr Frame(FrameType type, Address deviceAddress = DefaultDeviceAddress)
        : deviceAddress{deviceAddress}
        , type{type} {}

    constexpr Frame(compat::span<u8> data)
        : deviceAddress{data[0]}
        , type{static_cast<FrameType>(data[2])} {}

    FrameData GetData() const {
        FrameData data{};
        size_t payloadSize{WritePayload(compat::span<u8>(data.data() + HeaderSize, PayloadSizeMax))},
            frameSize{FrameTypeSize + payloadSize + FrameCrcSize};
        data.size = FrameSizeNotCountedBytes + frameSize;

        *reinterpret_cast<Header*>(data.data()) = Header{
            .deviceAddress = deviceAddress,
            .frameSize = static_cast<u8>(frameSize),
            .type = type,
        };
        data[data.size - 1] = CrsfCrc.Calculate(data.data() + FrameSizeNotCountedBytes, FrameTypeSize + payloadSize);

        return data;
    }

    virtual size_t WritePayload(compat::span<u8> payload) const {
        throw Exception("WritePayload not implemented for frame type: 0x{:X}", static_cast<uint8_t>(type));
    }
};

struct BatterySensorFrame : public Frame {
    static constexpr FrameType Type{FrameType::BatterySensor};

    BatterySensorData battery;

    constexpr BatterySensorFrame(BatterySensorData battery, Address deviceAddress = DefaultDeviceAddress)
        : Frame(Type, deviceAddress)
        , battery{battery} {}

    inline BatterySensorFrame(compat::span<u8> data)
        : Frame(data)
        , battery{*(reinterpret_cast<BatterySensorData*>(data.data() + HeaderSize))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        *reinterpret_cast<BatterySensorData*>(payload.data()) = battery;
        return sizeof(BatterySensorData);
    }
};

struct LinkStatisticsFrame : public Frame {
    static constexpr FrameType Type{FrameType::LinkStatistics};

    LinkStatisticsData statistics;

    constexpr LinkStatisticsFrame(LinkStatisticsData statistics, Address deviceAddress = DefaultDeviceAddress)
        : Frame(Type, deviceAddress)
        , statistics{statistics} {}

    inline LinkStatisticsFrame(compat::span<u8> data)
        : Frame(data)
        , statistics{*(reinterpret_cast<LinkStatisticsData*>(data.data() + HeaderSize))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        *reinterpret_cast<LinkStatisticsData*>(payload.data()) = statistics;
        return sizeof(LinkStatisticsData);
    }
};

struct RcChannelsFrame : public Frame {
    static constexpr FrameType Type{FrameType::RcChannelsPacked};

    RcChannelsData channels;

    constexpr RcChannelsFrame(RcChannelsData channels, Address deviceAddress = DefaultDeviceAddress)
        : Frame(Type, deviceAddress)
        , channels{channels} {}

    RcChannelsFrame(compat::span<u8> data)
        : Frame(data)
        , channels{*(reinterpret_cast<RcChannelsData*>(data.data() + HeaderSize))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        *reinterpret_cast<RcChannelsData*>(payload.data()) = channels;
        return sizeof(RcChannelsData);
    }
};

struct ExtFrame : public Frame {
    Address destinationAddress;
    Address originAddress;

    constexpr ExtFrame(FrameType type, Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : Frame(type, deviceAddress)
        , destinationAddress{destinationAddress}
        , originAddress{originAddress} {}

    ExtFrame(compat::span<u8> data)
        : Frame(data)
        , destinationAddress{data[3]}
        , originAddress{data[4]} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        auto address{reinterpret_cast<Address*>(payload.data())};
        *address = destinationAddress;
        *(address + 1) = originAddress;
        return sizeof(Address) * 2;
    }
};

struct PingFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::DevicePing};

    constexpr PingFrame(Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : ExtFrame(Type, destinationAddress, originAddress, deviceAddress) {}

    PingFrame(compat::span<u8> data)
        : ExtFrame(data) {}
};

#pragma pack(push, 1)
struct DeviceMetadata {
    u32 serialNo;
    u32 hardwareVersion;
    u32 softwareVersion;
    u8 fieldCount;
    u8 parameterVersion;

    constexpr bool operator==(const DeviceMetadata& other) const {
        return serialNo == other.serialNo && hardwareVersion == other.hardwareVersion && softwareVersion == other.softwareVersion && fieldCount == other.fieldCount && parameterVersion == other.parameterVersion;
    }
    
    constexpr bool operator!=(const DeviceMetadata& other) const {
        return !(*this == other);
    }
};
static_assert(sizeof(DeviceMetadata) == 14);
#pragma pack(pop)

struct DeviceInfoFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::DeviceInfo};

    std::string deviceName;
    DeviceMetadata deviceMetadata;

    DeviceInfoFrame(std::string deviceName, u32 serialNo, u32 hardwareVersion, u32 softwareVersion, u8 fieldCount, u8 parameterVersion, Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : ExtFrame(Type, destinationAddress, originAddress, deviceAddress)
        , deviceName{deviceName}
        , deviceMetadata{serialNo, hardwareVersion, softwareVersion, fieldCount, parameterVersion} {}

    DeviceInfoFrame(compat::span<u8> data)
        : ExtFrame(data)
        , deviceName{reinterpret_cast<char*>(data.data() + ExtSize)}
        , deviceMetadata{*(reinterpret_cast<DeviceMetadata*>(data.data() + ExtSize + deviceName.size() + 1))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        size_t offset{ExtFrame::WritePayload(payload)};

        std::copy(deviceName.begin(), deviceName.end(), payload.begin() + offset);
        payload[deviceName.size()] = '\0';
        offset += deviceName.size() + 1;

        std::memcpy(payload.data() + offset, &deviceMetadata, sizeof(DeviceMetadata));
        offset += sizeof(DeviceMetadata);

        return offset;
    }
};

struct ParameterSettingsEntryFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::ParameterSettingsEntry};

    u8 fieldIndex;
    u8 chunksRemaining;
    StaticVector<u8, PayloadSizeMax> data;

    ParameterSettingsEntryFrame(compat::span<u8> data)
        : ExtFrame(data)
        , fieldIndex{data[ExtSize]}
        , chunksRemaining{data[ExtSize + 1]}
        , data{data.begin() + ExtSize + 2, data.end() - FrameCrcSize} {}
};

struct ParameterReadFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::ParameterRead};

    u8 fieldIndex;
    u8 chunkIndex;

    constexpr ParameterReadFrame(u8 fieldIndex, u8 chunkIndex, Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : ExtFrame(Type, destinationAddress, originAddress, deviceAddress)
        , fieldIndex{fieldIndex}
        , chunkIndex{chunkIndex} {}

    ParameterReadFrame(compat::span<u8> data)
        : ExtFrame(data)
        , fieldIndex{data[ExtSize]}
        , chunkIndex{data[ExtSize + 1]} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        size_t offset{ExtFrame::WritePayload(payload)};

        payload[offset] = fieldIndex;
        payload[offset + 1] = chunkIndex;

        return offset + 2;
    }
};

struct ParameterWriteFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::ParameterWrite};
    using WriteValue = u8;

    u8 fieldIndex;
    u8 value;

    constexpr ParameterWriteFrame(u8 fieldIndex, u8 value, Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : ExtFrame(Type, destinationAddress, originAddress, deviceAddress)
        , fieldIndex{fieldIndex}
        , value{value} {}

    ParameterWriteFrame(compat::span<u8> data)
        : ExtFrame(data)
        , fieldIndex{data[ExtSize]}
        , value{data[ExtSize + 1]} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        size_t offset{ExtFrame::WritePayload(payload)};

        payload[offset] = fieldIndex;
        payload[offset + 1] = value;

        return offset + 2;
    }
};

struct OpenTxSyncData {
    i32 packetRate;
    i32 packetOffset;
};

struct RadioIdFrame : public ExtFrame {
    static constexpr FrameType Type{FrameType::RadioId};

    OpenTxSyncData sync;

    constexpr RadioIdFrame(OpenTxSyncData sync, Address destinationAddress, Address originAddress = DefaultOriginAddress, Address deviceAddress = DefaultDeviceAddress)
        : ExtFrame(Type, destinationAddress, originAddress, deviceAddress)
        , sync{sync} {}

    RadioIdFrame(compat::span<u8> data)
        : ExtFrame(data) {
        FrameType extendedType{data[ExtSize]};
        if (extendedType != FrameType::OpenTxSync)
            throw Exception("Unknown extended type: 0x{:X}", static_cast<int>(extendedType));

        auto syncData{reinterpret_cast<OpenTxSyncData*>(data.data() + ExtSize + 1)};
        sync = {
            .packetRate = static_cast<i32>(compat::byteswap32(static_cast<uint32_t>(syncData->packetRate))),
            .packetOffset = static_cast<i32>(compat::byteswap32(static_cast<uint32_t>(syncData->packetOffset))),
        };
    }

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        size_t offset{ExtFrame::WritePayload(payload)};

        payload[offset] = static_cast<u8>(FrameType::OpenTxSync);
        *reinterpret_cast<OpenTxSyncData*>(payload.data() + offset + 1) = {
            .packetRate = static_cast<i32>(compat::byteswap32(static_cast<uint32_t>(sync.packetRate))),
            .packetOffset = static_cast<i32>(compat::byteswap32(static_cast<uint32_t>(sync.packetOffset))),
        };

        return offset + 1 + sizeof(OpenTxSyncData);
    }
};

struct AttitudeFrame : public Frame {
    static constexpr FrameType Type{FrameType::Attitude};

    AttitudeData attitude;

    constexpr AttitudeFrame(AttitudeData attitude, Address deviceAddress = DefaultDeviceAddress)
        : Frame(Type, deviceAddress)
        , attitude{attitude} {}

    AttitudeFrame(compat::span<u8> data)
        : Frame(data)
        , attitude{*(reinterpret_cast<AttitudeData*>(data.data() + HeaderSize))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        *reinterpret_cast<AttitudeData*>(payload.data()) = attitude;
        return sizeof(AttitudeData);
    }
};

struct FlightModeFrame : public Frame {
    static constexpr FrameType Type{FrameType::FlightMode};

    FlightModeData flightMode;

    constexpr FlightModeFrame(FlightModeData flightMode, Address deviceAddress = DefaultDeviceAddress)
        : Frame(Type, deviceAddress)
        , flightMode{flightMode} {}

    FlightModeFrame(compat::span<u8> data)
        : Frame(data)
        , flightMode{*(reinterpret_cast<FlightModeData*>(data.data() + HeaderSize))} {}

    virtual size_t WritePayload(compat::span<u8> payload) const override {
        *reinterpret_cast<FlightModeData*>(payload.data()) = flightMode;
        return sizeof(FlightModeData);
    }
};

using FrameV = std::variant<BatterySensorFrame, LinkStatisticsFrame, RcChannelsFrame, PingFrame, DeviceInfoFrame, ParameterSettingsEntryFrame, ParameterReadFrame, ParameterWriteFrame, RadioIdFrame, AttitudeFrame, FlightModeFrame>;

inline FrameV GetFrame(compat::span<u8> data, FrameType type) {
    return [&]<size_t... Is>(std::index_sequence<Is...>) -> FrameV {
        std::optional<FrameV> result{};
        auto TryGetFrame{[&](auto index) -> bool {
            using T = std::variant_alternative_t<index, FrameV>;
            if (T::Type == type) {
                result = T(data);
                return true;
            }
            return false;
        }};
        if (!(TryGetFrame(std::integral_constant<size_t, Is>{}) || ...))
            throw Exception("Unknown frame type: 0x{:X}", static_cast<uint8_t>(type));
        if (!result)
            throw Exception("Failed to create frame of type: 0x{:X}", static_cast<uint8_t>(type));
        return *result;
    }(std::make_index_sequence<std::variant_size_v<FrameV>>{});
}
} // namespace crsf