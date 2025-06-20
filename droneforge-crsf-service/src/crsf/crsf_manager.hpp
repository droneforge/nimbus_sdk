/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 * Copyright © DroneForge
 */
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <optional>
#include <chrono>
#include <unordered_map>
#include <string>
#include <atomic>

#include "../common/compatibility.h"
#include "crsf.h"
#include "protocol.h"
#include "frame.h"

namespace droneforge::communication {

/**
 * @brief Event callbacks for CRSF communication events
 * 
 * These callbacks allow subscribers to receive notifications about
 * various CRSF protocol events without tight coupling.
 */
struct CrsfEventCallbacks {
    /// Called when link statistics are received
    std::function<void(const crsf::LinkStatisticsData&)> onLinkStats;
    
    /// Called when battery telemetry is received
    std::function<void(const crsf::BatterySensorData&)> onBattery;
    
    /// Called when attitude data is received
    std::function<void(const crsf::AttitudeData&)> onAttitude;
    
    /// Called when flight mode information is received
    std::function<void(const std::string&)> onFlightMode;
    
    /// Called when a device is discovered or updated
    std::function<void(const std::string& deviceName, crsf::Address address)> onDeviceInfo;
    
    /// Called when communication errors occur
    std::function<void(const std::string& error)> onError;
    
    /// Called when connection status changes
    std::function<void(bool connected)> onConnectionStatus;
    
    /// Called with thread performance metrics (threadName, processingTimeMs)
    std::function<void(const std::string& threadName, float processingTimeMs)> onPerformanceMetrics;
};

/**
 * @brief Manages CRSF protocol communication with drone devices
 * 
 * This class handles all aspects of CRSF communication including:
 * - Device discovery and management
 * - Bidirectional frame transmission/reception
 * - Telemetry data processing
 * - Connection monitoring and recovery
 * 
 * The manager runs two background threads:
 * - RX thread: Receives and processes incoming CRSF frames
 * - TX thread: Sends outgoing frames and manages ping/discovery
 */
class CrsfManager {
public:
    /**
     * @brief Constructs a new CRSF Manager
     * 
     * @param portName Serial port name/path (e.g., "COM1", "/dev/ttyUSB0")
     * @param baudRate Serial communication baud rate
     * @throws std::runtime_error if serial port cannot be opened
     */
    CrsfManager(const std::string& portName, size_t baudRate);
    
    /**
     * @brief Destructor - ensures clean shutdown of threads and resources
     */
    ~CrsfManager();
    
    // Non-copyable, non-movable for thread safety
    CrsfManager(const CrsfManager&) = delete;
    CrsfManager& operator=(const CrsfManager&) = delete;
    CrsfManager(CrsfManager&&) = delete;
    CrsfManager& operator=(CrsfManager&&) = delete;
    
    /**
     * @brief Sets event callbacks for communication events
     * 
     * @param callbacks Structure containing callback functions
     */
    void setEventCallbacks(const CrsfEventCallbacks& callbacks);
    
    /**
     * @brief Sends RC channel data to the flight controller
     * 
     * @param channels RC channel data to transmit
     */
    void sendRcChannels(const crsf::RcChannelsData& channels);
    
    /**
     * @brief Gets a read-only view of discovered devices
     * 
     * @return const std::vector<crsf::Device>& Vector of discovered devices
     */
    const std::vector<crsf::Device>& getDevices() const;
    
    /**
     * @brief Checks if the communication interface is active
     * 
     * @return true if serial port is open and operational
     */
    bool isActive();
    
    /**
     * @brief Requests graceful shutdown of communication threads
     */
    void requestStop();
    
    /**
     * @brief Checks if a stop has been requested
     * 
     * @return true if stop has been requested due to error or manual request
     */
    bool isStopRequested() const;
    
    /**
     * @brief Starts the CRSF communication threads
     * 
     * This allows for deferred thread startup after heavy initialization
     * is complete to avoid interference with serial communication.
     */
    void startThreads();

private:
    /// Core CRSF protocol interface
    crsf::Interface crsfInterface;
    
    /// Thread synchronization
    mutable std::mutex stateMutex;
    std::condition_variable stateCV;
    
    /// Communication threads - always use std::thread for compatibility
    std::thread rxThread;
    std::thread txThread;
    std::atomic<bool> threadsShouldStop{false};
    
    /// Device management
    std::vector<crsf::Device> devices;
    
    /// Connection state
    bool earlyPing{false};
    size_t pingsSinceLastRx{0};
    bool stopRequested{false};
    
    /// Timing control
    static constexpr auto PingInterval = std::chrono::milliseconds(100);
    static constexpr auto WakeInterval = std::chrono::milliseconds(10);
    static constexpr auto RcSendInterval = std::chrono::milliseconds(10); // 100 Hz
    std::chrono::steady_clock::time_point wakeTime{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point pingTime{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point nextRcSendTime{std::chrono::steady_clock::now()};
    
    /// Error tracking
    bool recurringException{false};
    
    /// Pending outgoing frames
    std::optional<crsf::RcChannelsFrame> pendingChannelFrame;
    
    /// Event callbacks
    CrsfEventCallbacks eventCallbacks;
    
    /// Connection tracking
    struct {
        std::string receiver{};
        std::string transmitter{};
        bool receiverFound{false};
        bool transmitterFound{false};
        std::chrono::steady_clock::time_point lastPingTime{};
        bool isPinging{false};
    } connectedDevices;
    
    /**
     * @brief Finds a device by its CRSF address
     * 
     * @param address CRSF device address to search for
     * @return crsf::Device* Pointer to device if found, nullptr otherwise
     */
    crsf::Device* findDevice(crsf::Address address);
    
    /**
     * @brief Updates device information and notifies callbacks
     * 
     * @param deviceName Name of the device
     * @param address CRSF address of the device
     */
    void updateDeviceInfo(const std::string& deviceName, crsf::Address address);
    
    /**
     * @brief RX thread main loop - handles incoming CRSF frames
     */
    void rxThreadLoop();
    
    /**
     * @brief TX thread main loop - handles outgoing frames and pings
     */
    void txThreadLoop();
    
    /**
     * @brief Processes a received CRSF frame and dispatches to appropriate handlers
     * 
     * @param frame The received frame variant
     */
    void processReceivedFrame(const crsf::FrameV& frame);
    
    /**
     * @brief Safely calls an event callback with error handling
     * 
     * @tparam Func Callback function type
     * @tparam Args Argument types
     * @param callback The callback function to call
     * @param args Arguments to pass to the callback
     */
    template<typename Func, typename... Args>
    void safeCallback(const Func& callback, Args&&... args) const;
};

} // namespace droneforge::communication 