/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 * Copyright © DroneForge
 */
#include "crsf_manager.hpp"

#include <algorithm>
#include <fmt/core.h>
#include "../common/thread_name.h"

namespace droneforge::communication {

CrsfManager::CrsfManager(const std::string& portName, size_t baudRate)
    : crsfInterface{portName, baudRate} {
}

CrsfManager::~CrsfManager() {
    stopRequested = true;
    threadsShouldStop = true;
    stateCV.notify_all();
    
    if (rxThread.joinable()) {
        rxThread.join();
    }
    
    if (txThread.joinable()) {
        txThread.join();
    }
}

void CrsfManager::startThreads() {
    threadsShouldStop = false;
    rxThread = std::thread(&CrsfManager::rxThreadLoop, this);
    txThread = std::thread(&CrsfManager::txThreadLoop, this);
}

void CrsfManager::setEventCallbacks(const CrsfEventCallbacks& callbacks) {
    std::scoped_lock lock{stateMutex};
    eventCallbacks = callbacks;
}

void CrsfManager::sendRcChannels(const crsf::RcChannelsData& channels) {
    std::scoped_lock lock{stateMutex};
    pendingChannelFrame = crsf::RcChannelsFrame(channels, crsf::Address::FlightController);
    stateCV.notify_all();
}

const std::vector<crsf::Device>& CrsfManager::getDevices() const {
    std::scoped_lock lock{stateMutex};
    return devices;
}

bool CrsfManager::isActive() {
    return crsfInterface.IsActive() && !stopRequested;
}

void CrsfManager::requestStop() {
    std::scoped_lock lock{stateMutex};
    stopRequested = true;
    threadsShouldStop = true;
    stateCV.notify_all();
}

bool CrsfManager::isStopRequested() const {
    std::scoped_lock lock{stateMutex};
    return stopRequested;
}

crsf::Device* CrsfManager::findDevice(crsf::Address address) {
    auto it = std::find_if(devices.begin(), devices.end(), 
        [&](const crsf::Device& device) { 
            return device.address == address; 
        });
    return (it == devices.end()) ? nullptr : &*it;
}

void CrsfManager::updateDeviceInfo(const std::string& deviceName, crsf::Address address) {
    if (deviceName.find("RX") != std::string::npos) {
        connectedDevices.receiver = deviceName;
        connectedDevices.receiverFound = true;
    } else if (deviceName.find("Micro") != std::string::npos) {
        connectedDevices.transmitter = deviceName;
        connectedDevices.transmitterFound = true;
    }
    
    // Notify callback
    safeCallback(eventCallbacks.onDeviceInfo, deviceName, address);
}

void CrsfManager::rxThreadLoop() {
    SetThreadName("CRSF RX");
    
    // Track error frequency to prevent log spam
    int consecutiveErrors = 0;
    auto lastErrorTime = std::chrono::steady_clock::now();
    std::unordered_map<std::string, int> errorCounts;
    
    while (!threadsShouldStop && !stopRequested) {
        auto startTime = std::chrono::steady_clock::now();
        
        try {
            // Use a shorter timeout to check stop token more frequently
            std::optional<crsf::FrameV> frameV{
                crsfInterface.ReceiveFrame(std::chrono::steady_clock::now() + std::chrono::milliseconds{50})
            };
            
            // Reset error counter on successful operation
            if (consecutiveErrors > 0) {
                auto timeSinceLastError = std::chrono::steady_clock::now() - lastErrorTime;
                if (timeSinceLastError > std::chrono::seconds(5)) {
                    consecutiveErrors = 0;
                    errorCounts.clear();
                }
            }
            
            // If no frame was received, just continue
            if (!frameV) {
                continue;
            }

            // Process received frame
            {
                std::scoped_lock lock{stateMutex};
                try {
                    processReceivedFrame(*frameV);
                    pingsSinceLastRx = 0;
                } catch (const std::exception& e) {
                    // Don't let frame processing errors crash the RX thread
                    fmt::print("CRSF Frame processing error: {}\n", e.what());
                    safeCallback(eventCallbacks.onError, 
                        fmt::format("Frame processing error: {}", e.what()));
                    // Continue processing other frames
                }
            }
            
            stateCV.notify_all();
            
        } catch (std::exception& e) {
            std::string errorMsg = e.what();
            
            // Track errors to avoid log spam
            consecutiveErrors++;
            lastErrorTime = std::chrono::steady_clock::now();
            errorCounts[errorMsg]++;
            
            // Only log after error count threshold or at intervals
            bool shouldLog = false;
            if (consecutiveErrors <= 5) {
                // Log first few errors immediately
                shouldLog = true;
            } else if (consecutiveErrors % 50 == 0) {
                // Log every 50th error after that
                shouldLog = true;
            }
            
            if (shouldLog) {
                fmt::print("CRSF RX Error: {} (count: {})\n", errorMsg, errorCounts[errorMsg]);
                safeCallback(eventCallbacks.onError, 
                    fmt::format("RX Error: {} (count: {})", errorMsg, errorCounts[errorMsg]));
            }
            
            // Check if connection is still active
            if (!crsfInterface.IsActive()) {
                fmt::print("Serial port is inactive, stopping CRSF RX thread\n");
                std::scoped_lock lock{stateMutex};
                stopRequested = true;
                safeCallback(eventCallbacks.onConnectionStatus, false);
                return;
            }
            
            // Add small delay after errors to avoid CPU spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            
            // Reset interface if there are too many errors
            if (consecutiveErrors > 1000) {
                fmt::print("Too many consecutive CRSF errors, trying to reset interface\n");
                try {
                    crsfInterface.ResetDevice();
                    consecutiveErrors = 0;
                    errorCounts.clear();
                } catch (...) {
                    // If reset fails, continue with errors
                }
            }
        }
        
        // Track performance metrics
        auto endTime = std::chrono::steady_clock::now();
        float processingTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
        safeCallback(eventCallbacks.onPerformanceMetrics, "RX", processingTimeMs);
        
        // Rate limiting
        auto processingTime = endTime - startTime;
        if (processingTime < std::chrono::milliseconds(5)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5) - processingTime);
        }
    }
}

void CrsfManager::txThreadLoop() {
    SetThreadName("CRSF TX");
    
    // Add a startup delay to give the device time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Track connection attempts
    int connectionAttempts = 0;
    const int maxConnectionAttempts = 5;
    
    while (!threadsShouldStop && !stopRequested) {
        auto startTime = std::chrono::steady_clock::now();
        
        try {
            std::unique_lock<std::mutex> lock{stateMutex};
            auto nextWakeTime = std::min<std::chrono::steady_clock::time_point>(wakeTime, pingTime);
            stateCV.wait_until(lock, nextWakeTime);

            try {
                if (pingsSinceLastRx >= 8) {
                    if (++connectionAttempts <= maxConnectionAttempts) {
                        fmt::print("No CRSF packets received after {} pings, resetting device (attempt {}/{})\n", 
                            pingsSinceLastRx, connectionAttempts, maxConnectionAttempts);
                        crsfInterface.ResetDevice();
                        pingsSinceLastRx = 0;
                        earlyPing = true;
                        // Add a longer delay after reset to allow the device to recover
                        lock.unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        continue;
                    } else {
                        fmt::print("Maximum CRSF connection attempts reached ({}). Continuing in limited mode.\n", 
                            maxConnectionAttempts);
                        // Instead of repeatedly trying to reset, just clear the counter and continue
                        pingsSinceLastRx = 0;
                    }
                }

                // Send pending device frames
                for (crsf::Device& device : devices) {
                    auto frame = device.TryGetTxFrame();
                    if (frame) {
                        crsfInterface.SendFrame(*frame);
                    } else if (device.address == crsf::Address::CrsfTransmitter && pendingChannelFrame) {
                        // Only send RC channels when we have a discovered transmitter device
                        auto nowSend = std::chrono::steady_clock::now();
                        if (nowSend >= nextRcSendTime) {
                            crsfInterface.SendFrame(*pendingChannelFrame);
                            nextRcSendTime = nowSend + RcSendInterval;
                        }
                    }
                }
                
                auto now = std::chrono::steady_clock::now();
                if (now >= pingTime) {
                    pingTime = now + PingInterval;
                    pingsSinceLastRx++;

                    if (earlyPing || devices.empty()) {
                        earlyPing = false;
                        for (auto address : crsf::DeviceAddressList) {
                            crsfInterface.SendFrame(crsf::PingFrame{address});
                        }
                    }
                    
                    // Update ping status
                    connectedDevices.lastPingTime = now;
                    connectedDevices.isPinging = true;
                }

                if (now >= wakeTime) {
                    wakeTime = now + WakeInterval;
                }
            } catch (const std::exception& e) {
                fmt::print("CRSF TX inner Error: {}\n", e.what());
                safeCallback(eventCallbacks.onError, fmt::format("TX inner Error: {}", e.what()));
            }
            
            // Reset this flag if we've successfully completed a loop
            if (recurringException) {
                recurringException = false;
            }
            
        } catch (const std::exception& e) {
            fmt::print("CRSF TX Error: {}\n", e.what());
            safeCallback(eventCallbacks.onError, fmt::format("TX Error: {}", e.what()));
            
            // If we're getting recurring exceptions, attempt recovery
            if (recurringException) {
                try {
                    fmt::print("Recurring CRSF TX errors, attempting recovery...\n");
                    crsfInterface.ResetDevice();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    recurringException = false;
                } catch (...) {
                    // If recovery fails, stop TX thread
                    std::scoped_lock lock{stateMutex};
                    stopRequested = true;
                    safeCallback(eventCallbacks.onConnectionStatus, false);
                    break;
                }
            } else {
                recurringException = true;
            }
            
            if (!crsfInterface.IsActive()) {
                fmt::print("Serial port is inactive, stopping CRSF TX thread\n");
                std::scoped_lock lock{stateMutex};
                stopRequested = true;
                safeCallback(eventCallbacks.onConnectionStatus, false);
                break;
            }
            
            // Add a delay after exceptions to avoid CPU spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Track performance metrics
        auto endTime = std::chrono::steady_clock::now();
        float processingTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
        safeCallback(eventCallbacks.onPerformanceMetrics, "TX", processingTimeMs);
        
        // Rate limiting
        auto processingTime = endTime - startTime;
        if (processingTime < std::chrono::milliseconds(5)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5) - processingTime);
        }
    }
}

void CrsfManager::processReceivedFrame(const crsf::FrameV& frame) {
    std::visit([&](auto&& frame) {
        using T = std::decay_t<decltype(frame)>;

        if constexpr (std::is_same_v<T, crsf::BatterySensorFrame>) {
            safeCallback(eventCallbacks.onBattery, frame.battery);
        } else if constexpr (std::is_same_v<T, crsf::LinkStatisticsFrame>) {
            safeCallback(eventCallbacks.onLinkStats, frame.statistics);
        } else if constexpr (std::is_same_v<T, crsf::PingFrame>) {
            // Ping frame received
        } else if constexpr (std::is_same_v<T, crsf::DeviceInfoFrame>) {
            crsf::Device* device = findDevice(frame.originAddress);
            if (device) {
                device->Update(frame.deviceName, frame.deviceMetadata);
                updateDeviceInfo(frame.deviceName, frame.originAddress);
            } else {
                device = &devices.emplace_back(frame.originAddress, frame.deviceName, frame.deviceMetadata);
                updateDeviceInfo(frame.deviceName, frame.originAddress);
            }
        } else if constexpr (std::is_same_v<T, crsf::ParameterSettingsEntryFrame>) {
            auto device = findDevice(frame.originAddress);
            if (!device) {
                earlyPing = true;
                fmt::print("CRSF Device not found: 0x{:02X}\n", static_cast<int>(frame.originAddress));
                return;
            }

            try {
                crsf::Parameter* parameter = device->FindParameter(frame.fieldIndex);
                if (parameter) {
                    if (parameter->complete) {
                        return;
                    }
                    parameter->ProcessChunk(compat::span<u8>(const_cast<StaticVector<u8, 62>&>(frame.data).data(), frame.data.size), frame.chunksRemaining);
                } else {
                    // Don't spam logs for missing parameters - this is normal during discovery
                    // fmt::print("CRSF Parameter not found: {}\n", frame.fieldIndex);
                    return;
                }
            } catch (const std::exception& e) {
                // Make parameter processing non-fatal - just log and continue
                fmt::print("CRSF Parameter processing error (field {}): {}\n", frame.fieldIndex, e.what());
                safeCallback(eventCallbacks.onError, 
                    fmt::format("Parameter processing error (field {}): {}", frame.fieldIndex, e.what()));
                return;
            }
        } else if constexpr (std::is_same_v<T, crsf::RadioIdFrame>) {
            auto device = findDevice(frame.originAddress);
            if (!device) {
                earlyPing = true;
                fmt::print("CRSF Device not found: 0x{:02X}\n", static_cast<int>(frame.originAddress));
                return;
            }

            device->syncData = frame.sync;
        } else if constexpr (std::is_same_v<T, crsf::AttitudeFrame>) {
            // Store attitude data (high frequency telemetry)
            safeCallback(eventCallbacks.onAttitude, frame.attitude);
        } else if constexpr (std::is_same_v<T, crsf::FlightModeFrame>) {
            // Process flight mode information
            std::string mode(frame.flightMode.flightMode.data());
            safeCallback(eventCallbacks.onFlightMode, mode);
        } else {
            fmt::print("Unknown CRSF frame type: 0x{:02X}\n", static_cast<uint8_t>(frame.type));
            safeCallback(eventCallbacks.onError, 
                fmt::format("Unknown frame type: 0x{:02X}", static_cast<uint8_t>(frame.type)));
        }
    }, frame);
}

template<typename Func, typename... Args>
void CrsfManager::safeCallback(const Func& callback, Args&&... args) const {
    if (callback) {
        try {
            callback(std::forward<Args>(args)...);
        } catch (const std::exception& e) {
            fmt::print("Error in CRSF callback: {}\n", e.what());
        } catch (...) {
            fmt::print("Unknown error in CRSF callback\n");
        }
    }
}

} // namespace droneforge::communication 