/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#pragma once

#include <chrono>
#include <string>
#include <variant>
#include <vector>

#include "../common/error.h"
#include "../common/compatibility.h"
#include <fmt/format.h>

#include "protocol.h"

namespace crsf {
template <typename T>
struct IntegerParameter {
    T value;
    T min;
    T max;
    std::string units;

    IntegerParameter(compat::span<u8> data) {
        auto ptr{data.data()};
        
        value = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);
        min = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);
        max = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct FloatParameter {
    float value;
    float min;
    float max;
    u8 precision;
    u32 step;
    std::string units;

    FloatParameter(compat::span<u8> data) {
        auto ptr{data.data()};

        value = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);
        min = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);
        max = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);

        precision = *ptr++;
        step = *reinterpret_cast<u32*>(ptr);
        ptr += sizeof(u32);

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct TextSelectionParameter {
    u8 selectedIndex;
    u8 minIndex;
    u8 maxIndex;
    u8 defaultIndex;
    std::vector<std::string> options;
    std::string units;

    TextSelectionParameter(compat::span<u8> data) {
        auto ptr{data.data()};
        std::string_view optionsString{reinterpret_cast<char*>(ptr)};
        for (size_t start{}, end{}; end != std::string_view::npos; start = end + 1) {
            end = optionsString.find(';', start);
            options.emplace_back(optionsString.substr(start, end - start));
        }
        ptr += optionsString.size() + 1;

        constexpr char LuaSymbolArrowUp{'\xc0'};
        constexpr char LuaSymbolArrowDown{'\xc1'};

        for (std::string& option : options) {
            for (size_t i{}; i < option.size(); i++) {
                if (option[i] == LuaSymbolArrowUp) {
                    option.replace(i, 1, "↑");
                    i++;
                } else if (option[i] == LuaSymbolArrowDown) {
                    option.replace(i, 1, "↓");
                    i++;
                }
            }
        }

        selectedIndex = *ptr++;
        minIndex = *ptr++;
        maxIndex = *ptr++;
        defaultIndex = *ptr++;

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct FolderParameter {
    std::string displayName;

    FolderParameter(compat::span<u8> data) {
        if (!data.empty())
            displayName = std::string(reinterpret_cast<char*>(data.data()));
    }
};

struct InfoParameter {
    std::string display;

    InfoParameter(compat::span<u8> data) {
        display = std::string(reinterpret_cast<char*>(data.data()));
    }
};

struct CommandParameter {
    CommandStep step;
    std::string status;

    CommandParameter(compat::span<u8> data) {
        step = static_cast<CommandStep>(data[0]);
        status = std::string(reinterpret_cast<char*>(data.data() + 1));
    }
};

using ParameterValue = std::variant<std::monostate, IntegerParameter<u8>, IntegerParameter<i8>, IntegerParameter<u16>, IntegerParameter<i16>, IntegerParameter<u32>, IntegerParameter<i32>, FloatParameter, TextSelectionParameter, FolderParameter, InfoParameter, CommandParameter>;

inline ParameterValue GetParameterValue(compat::span<u8> data, ParameterType type) {
    switch (type) {
        case ParameterType::Uint8:
            return IntegerParameter<u8>(data);
        case ParameterType::Int8:
            return IntegerParameter<i8>(data);
        case ParameterType::Uint16:
            return IntegerParameter<u16>(data);
        case ParameterType::Int16:
            return IntegerParameter<i16>(data);
        case ParameterType::Uint32:
            return IntegerParameter<u32>(data);
        case ParameterType::Int32:
            return IntegerParameter<i32>(data);
        case ParameterType::Float:
            return FloatParameter(data);
        case ParameterType::TextSelection:
            return TextSelectionParameter(data);
        case ParameterType::Folder:
            return FolderParameter(data);
        case ParameterType::Info:
            return InfoParameter(data);
        case ParameterType::Command:
            return CommandParameter(data);
        default:
            throw Exception("Unknown parameter type: {}", static_cast<int>(type));
    }
}

/**
 * @brief An abstraction over a CRSF parameter, processing every chunk of data until the parameter is complete.
 */
struct Parameter {
    u8 fieldIndex{};
    u8 parentIndex{};
    u8 chunkIndex{};
    i16 lastChunksRemaining{-1};
    bool complete{};
    bool requestedChunk{};
    bool activeCommand{};
    std::chrono::time_point<std::chrono::steady_clock> requestTime{};
    std::chrono::time_point<std::chrono::steady_clock> updateTime{};

    std::string name{};
    ParameterType type{};
    std::vector<u8> rawData;
    ParameterValue value{};

#pragma pack(push, 1)
    struct FirstChunkData {
        u8 parentIndex;
        ParameterType type;
    };
    static_assert(sizeof(FirstChunkData) == 2);
#pragma pack(pop)

    Parameter(u8 fieldIndex)
        : fieldIndex{fieldIndex} {}

    void ProcessChunk(compat::span<u8> chunk, u8 chunksRemaining) {
        // Handle chunk order mismatches
        if (lastChunksRemaining > 0 && chunksRemaining != lastChunksRemaining - 1) {
            static int mismatchCounter = 0;
            static auto lastLogTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            
            // Rate-limited logging to avoid spam
            bool shouldLog = false;
            if (mismatchCounter < 5) {
                shouldLog = true;
            } else if (mismatchCounter % 50 == 0) {
                shouldLog = true;
            } else if (std::chrono::duration_cast<std::chrono::seconds>(now - lastLogTime).count() > 10) {
                shouldLog = true;
                lastLogTime = now;
            }
            
            if (shouldLog) {
                fmt::print("Parameter chunk order mismatch (field {}): expected {}, got {} (count: {})\n", 
                    fieldIndex, lastChunksRemaining - 1, chunksRemaining, ++mismatchCounter);
            } else {
                ++mismatchCounter;
            }

            Reset();
        }
        
        lastChunksRemaining = chunksRemaining;
        rawData.insert(rawData.end(), chunk.begin(), chunk.end());
        chunkIndex++;
        requestedChunk = false;

        if (chunksRemaining == 0) {
            // Parameter is complete, parse the data
            try {
                auto ptr{rawData.data()};

                FirstChunkData* firstChunkData{reinterpret_cast<FirstChunkData*>(ptr)};
                parentIndex = firstChunkData->parentIndex;
                type = firstChunkData->type;
                ptr += sizeof(FirstChunkData);

                name = std::string(reinterpret_cast<char*>(ptr));
                ptr += name.size() + 1;
                
                value = GetParameterValue(compat::span<u8>(ptr, rawData.size() - (ptr - rawData.data())), type);
                complete = true;
                updateTime = std::chrono::steady_clock::now();

                if (activeCommand) {
                    CommandParameter& command{std::get<CommandParameter>(value)};
                    if (command.step == CommandStep::Idle)
                        activeCommand = false;
                }
            } catch (const std::exception& e) {
                // Unknown/unsupported parameter type – mark parameter as complete and continue without value
                fmt::print("Parameter parsing error (field {}): {}\n", fieldIndex, e.what());
                value = std::monostate{};
                complete = true;
                updateTime = std::chrono::steady_clock::now();
            }
            
            // Clear raw data after processing
            rawData.clear();
        }
    }

    void Reset() {
        chunkIndex = 0;
        lastChunksRemaining = -1;
        requestedChunk = false;
        rawData.clear();
        complete = false;
        if (!activeCommand)
            value = std::monostate{};
    }
};
}