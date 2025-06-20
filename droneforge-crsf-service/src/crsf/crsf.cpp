/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#include "protocol.h"
#include <chrono>
#include <thread>
#include <algorithm>

#include "crsf.h"

namespace crsf {
bool Interface::IsActive() noexcept {
    try {
        return serial.IsOpen();
    } catch (std::exception& e) {
        return false;
    }
}

void Interface::ResetDevice() noexcept { 
    try {
        if (!serial.IsOpen())
            return;

        serial.SetDTR(false);
        serial.SetRTS(false);
        serial.WriteByte(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial.SetDTR(true);
        serial.SetRTS(true);
        serial.WriteByte(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (std::exception& e) {}
}

void Interface::SendFrame(const FrameV& frame) {
    auto data{[&]() -> FrameData {
        return std::visit([&](auto&& frame) {
            return frame.GetData();
        },
            frame);
    }()};

    // Note: Any frames need to be sent twice to ensure they are received.
    serial.Write(data);
    serial.Write(data);
}

std::optional<FrameV> Interface::ReceiveFrame(std::chrono::time_point<std::chrono::steady_clock> timeout) {
    static auto lastFrameTime = std::chrono::steady_clock::now();
    static constexpr auto MinFrameInterval = std::chrono::milliseconds(2); // Throttle incoming frames
    
    // Throttle processing if frames are coming too quickly
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - lastFrameTime;
    if (elapsed < MinFrameInterval) {
        // Skip this frame if we're receiving too many too quickly
        ::std::this_thread::sleep_for(MinFrameInterval - elapsed);
    }
    
    FrameData frame{};
    bool readAddress{false}; //!< Whether the address has been read while looking for the sync byte.

    // Limit how long we search for sync bytes to avoid blocking
    auto syncSearchTimeout = std::min<std::chrono::time_point<std::chrono::steady_clock>>(
        timeout, now + std::chrono::milliseconds(20));
    
    while (std::chrono::steady_clock::now() < syncSearchTimeout) {
        try {
            auto optionalByte{serial.ReadByte(syncSearchTimeout)};
            if (!optionalByte)
                return std::nullopt;

            u8 byte{*optionalByte};
            if (byte == SyncByte) {
                break;
            } else if (static_cast<Address>(byte) == Address::CrsfTransmitter
                    || static_cast<Address>(byte) == Address::RadioTransmitter) {
                readAddress = true;
                frame[0] = byte;
                break;
            } else {
                // Limit verbose logging to minimize processing overhead
                static int dropCounter = 0;
                if (++dropCounter % 100 == 0) {
                    if (byte >= 0x20 && byte <= 0x7E)
                        fmt::print("Dropping: '{}' (and others)\n", static_cast<char>(byte));
                    else
                        fmt::print("Dropping: 0x{:02X} (and others)\n", static_cast<int>(byte));
                }
            }
        } catch (const std::exception& e) {
            // Tolerate temporary read errors and retry
            ::std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // If we exited the loop without finding sync, return empty
    if (std::chrono::steady_clock::now() >= syncSearchTimeout) {
        return std::nullopt;
    }

    try {
        // Read header
        Header* header{reinterpret_cast<Header*>(frame.data_resize(sizeof(Header)))};
        serial.Read(compat::span<u8>(frame.data() + readAddress, sizeof(Header) - readAddress));

        // Validate frame size
        if (header->frameSize + FrameSizeNotCountedBytes > FrameSizeMax)
            throw Exception("Frame size too large: 0x{:X}", header->frameSize);
        else if (header->frameSize < FrameTypeSize + FrameCrcSize)
            throw Exception("Frame size too small: 0x{:X}", header->frameSize);

        // Read rest of frame
        frame.resize(header->frameSize + FrameSizeNotCountedBytes);
        serial.Read(compat::span<u8>(frame.data() + sizeof(Header), header->frameSize - FrameTypeSize));

        // Validate CRC
        u8 crc{CrsfCrc.Calculate(frame.data() + FrameSizeNotCountedBytes, header->frameSize - FrameCrcSize)};
        u8 frameCrc{frame[FrameSizeNotCountedBytes + header->frameSize - FrameCrcSize]};
        if (crc != frameCrc)
            throw Exception("CRC mismatch: expected 0x{:02X}, got 0x{:02X}", crc, frameCrc);

        // Update frame time for throttling
        lastFrameTime = std::chrono::steady_clock::now();
        
        // Skip duplicate frames
        if (frame == oldFrame)
            return ReceiveFrame(timeout);
        else
            oldFrame = frame;

        return GetFrame(compat::span<u8>(frame.data(), header->frameSize + FrameSizeNotCountedBytes), header->type);
    } catch (const std::exception& e) {
        // Tolerate errors when reading frame data
        ::std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return std::nullopt;
    }
}

Interface::Interface(std::string port, size_t baud)
    : serial(port, baud) {}

Device::Device(Address address, const std::string& name, DeviceMetadata metadata)
    : address{address}
    , name{name}
    , metadata{metadata} {
    for (u8 index{1}; index <= metadata.fieldCount; index++)
        parameters.emplace_back(Parameter{index});
}

void Device::Update(const std::string& pName, DeviceMetadata pMetadata) {
    if (name != pName || metadata != pMetadata) {
        name = pName;
        metadata = pMetadata;
        
        parameters.clear();
        for (u8 index{1}; index <= metadata.fieldCount; index++)
            parameters.emplace_back(Parameter{index});

        parameterWrites.clear();
        syncData.reset();
    }
}

Parameter* Device::FindParameter(u8 fieldIndex) {
    auto it{std::find_if(parameters.begin(), parameters.end(), [&](const Parameter& parameter) { return parameter.fieldIndex == fieldIndex; })};
    if (it == parameters.end())
        return nullptr;
    return &*it;
}

void Device::WriteParameter(u8 fieldIndex, ParameterWriteFrame::WriteValue value) {
    Parameter* parameter{FindParameter(fieldIndex)};
    if (!parameter || !parameter->complete)
        return;

    parameterWrites.emplace_back(ParameterWriteFrame{fieldIndex, value, address});

    if (parameter->type == ParameterType::Command) {
        parameter->activeCommand = true;
        parameter->Reset();
        return;
    }

    // We need to reset siblings along with the parent, when a parameter is written to since their values might be interdependent.
    // Any children, or siblings that are subfolders or commands will not be reset since they are not expected to have interdependent values.
    u8 parentIndex{parameter->parentIndex};
    for (Parameter& parameter : parameters)
        if (parameter.fieldIndex == fieldIndex || (parameter.parentIndex == parentIndex && parameter.type != ParameterType::Folder && parameter.type != ParameterType::Command))
            parameter.Reset();
}

std::optional<FrameV> Device::TryGetTxFrame() {
    if (!parameterWrites.empty()) {
        FrameV frame{parameterWrites.front()};
        parameterWrites.pop_front();
        return frame;
    }

    // Throttle parameter discovery to prevent serial port flooding
    static auto lastParameterRequest = std::chrono::steady_clock::now();
    constexpr auto ParameterRequestInterval = std::chrono::milliseconds(50); // 20 Hz max
    auto now = std::chrono::steady_clock::now();
    
    if (now - lastParameterRequest < ParameterRequestInterval) {
        return std::nullopt; // Too soon, skip this request
    }

    for (Parameter& parameter : parameters) {
        constexpr auto ParameterResendTimeout{std::chrono::milliseconds(200)}; // Increased timeout
        if (!parameter.complete) {
            if (parameter.requestedChunk && (now - parameter.requestTime) < ParameterResendTimeout)
                continue;
            parameter.requestedChunk = true;
            parameter.requestTime = now;
            lastParameterRequest = now; // Update throttle timer
            
            // Add detailed logging for parameter requests
            static bool enableRequestLogging = false;
            
            return ParameterReadFrame{parameter.fieldIndex, parameter.chunkIndex, address};
        }
    }

    return std::nullopt;
}
}