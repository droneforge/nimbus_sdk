/* MIT License
 * Copyright (c) 2024 DroneForge
 * 
 * Client interface for DroneForge CRSF Service telemetry data.
 * High-performance shared memory interface with sub-microsecond latency.
 */

#pragma once

#include <atomic>
#include <optional>
#include <chrono>
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace droneforge::telemetry {

struct AttitudeData {
    float pitch;  // degrees
    float roll;   // degrees
    float yaw;    // degrees
    uint64_t timestamp_us;
};

struct BatteryData {
    float voltage;  // volts
    float current;  // amps
    float capacity; // mAh
    uint8_t remaining_percent; // %
    uint64_t timestamp_us;
};

struct LinkData {
    float uplink_rssi;   // dBm
    float downlink_rssi; // dBm
    uint8_t link_quality; // %
    int8_t snr;          // dB
    uint64_t timestamp_us;
};

/**
 * @brief High-performance telemetry client using shared memory IPC
 * 
 * Provides sub-microsecond latency access to real-time telemetry data
 * from the DroneForge CRSF Service.
 */
class TelemetryClient {
public:
    TelemetryClient() = default;
    ~TelemetryClient();
    
    // Non-copyable
    TelemetryClient(const TelemetryClient&) = delete;
    TelemetryClient& operator=(const TelemetryClient&) = delete;
    
    /**
     * @brief Connect to the CRSF service shared memory
     * @return true if connection successful, false otherwise
     */
    bool connect();
    
    /**
     * @brief Disconnect from shared memory
     */
    void disconnect();
    
    /**
     * @brief Check if service is running and connected
     * @return true if service is active
     */
    bool isConnected() const;
    
    /**
     * @brief Get current attitude data (pitch, roll, yaw)
     * @return Attitude data if available and recent, nullopt otherwise
     */
    std::optional<AttitudeData> getAttitude() const;
    
    /**
     * @brief Get current battery telemetry
     * @return Battery data if available and recent, nullopt otherwise
     */
    std::optional<BatteryData> getBattery() const;
    
    /**
     * @brief Get current link statistics
     * @return Link data if available and recent, nullopt otherwise
     */
    std::optional<LinkData> getLink() const;
    
    /**
     * @brief Get error information from service
     * @return Error count and last error message
     */
    std::pair<uint32_t, std::string> getErrorInfo() const;
    
    /**
     * @brief Check if new data is available since last check
     * @return true if sequence number has changed
     */
    bool hasNewData();

private:
    // Internal shared memory structure (matches service)
    struct TelemetrySharedMemory {
        std::atomic<uint32_t> sequence_number{0};
        std::atomic<bool> attitude_valid{false};
        std::atomic<bool> battery_valid{false};
        std::atomic<bool> link_valid{false};
        
        struct {
            float pitch;
            float roll; 
            float yaw;
            uint64_t timestamp_us;
        } attitude;
        
        struct {
            float voltage;
            float current;
            float capacity;
            uint8_t remaining_percent;
            uint64_t timestamp_us;
        } battery;
        
        struct {
            float uplink_rssi;
            float downlink_rssi;
            uint8_t link_quality;
            int8_t snr;
            uint64_t timestamp_us;
        } link;
        
        std::atomic<bool> service_running{true};
        std::atomic<uint32_t> error_count{0};
        char last_error[256] = {};
    };
    
    TelemetrySharedMemory* shared_memory = nullptr;
    uint32_t last_sequence_number = 0;
    
#ifdef _WIN32
    HANDLE shm_handle = INVALID_HANDLE_VALUE;
#else
    int shm_fd = -1;
#endif
    
    /**
     * @brief Check if timestamp is recent (within last 100ms)
     */
    bool isRecentTimestamp(uint64_t timestamp_us) const;
    
    /**
     * @brief Get current timestamp in microseconds
     */
    uint64_t getCurrentTimestampMicros() const;
};

/**
 * @brief Singleton telemetry store for easy global access
 * 
 * This provides a convenient singleton interface for accessing telemetry
 * data throughout your application.
 */
class TelemetryStore {
public:
    static TelemetryStore& getInstance();
    
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    std::optional<AttitudeData> getAttitude() const;
    std::optional<BatteryData> getBattery() const;
    std::optional<LinkData> getLink() const;
    
    bool hasNewData();

private:
    TelemetryClient client;
};

} // namespace droneforge::telemetry

// Implementation
namespace droneforge::telemetry {

inline TelemetryClient::~TelemetryClient() {
    disconnect();
}

inline bool TelemetryClient::connect() {
    if (shared_memory) {
        return true; // Already connected
    }
    
    const char* shm_name = "droneforge_crsf_telemetry";
    
#ifdef _WIN32
    shm_handle = OpenFileMappingA(FILE_MAP_READ, FALSE, shm_name);
    if (shm_handle == nullptr) {
        return false;
    }
    
    shared_memory = static_cast<TelemetrySharedMemory*>(
        MapViewOfFile(shm_handle, FILE_MAP_READ, 0, 0, sizeof(TelemetrySharedMemory))
    );
    
    if (shared_memory == nullptr) {
        CloseHandle(shm_handle);
        shm_handle = INVALID_HANDLE_VALUE;
        return false;
    }
#else
    shm_fd = shm_open(shm_name, O_RDONLY, 0);
    if (shm_fd == -1) {
        return false;
    }
    
    shared_memory = static_cast<TelemetrySharedMemory*>(
        mmap(nullptr, sizeof(TelemetrySharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0)
    );
    
    if (shared_memory == MAP_FAILED) {
        close(shm_fd);
        shm_fd = -1;
        shared_memory = nullptr;
        return false;
    }
#endif
    
    return true;
}

inline void TelemetryClient::disconnect() {
#ifdef _WIN32
    if (shared_memory) {
        UnmapViewOfFile(shared_memory);
        shared_memory = nullptr;
    }
    if (shm_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(shm_handle);
        shm_handle = INVALID_HANDLE_VALUE;
    }
#else
    if (shared_memory && shared_memory != MAP_FAILED) {
        munmap(shared_memory, sizeof(TelemetrySharedMemory));
        shared_memory = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
#endif
}

inline bool TelemetryClient::isConnected() const {
    return shared_memory != nullptr && shared_memory->service_running.load();
}

inline uint64_t TelemetryClient::getCurrentTimestampMicros() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()
    ).count();
}

inline bool TelemetryClient::isRecentTimestamp(uint64_t timestamp_us) const {
    auto now = getCurrentTimestampMicros();
    return (now - timestamp_us) < 100000; // 100ms
}

inline std::optional<AttitudeData> TelemetryClient::getAttitude() const {
    if (!shared_memory || !shared_memory->attitude_valid.load()) {
        return std::nullopt;
    }
    
    AttitudeData data;
    data.pitch = shared_memory->attitude.pitch;
    data.roll = shared_memory->attitude.roll;
    data.yaw = shared_memory->attitude.yaw;
    data.timestamp_us = shared_memory->attitude.timestamp_us;
    
    if (!isRecentTimestamp(data.timestamp_us)) {
        return std::nullopt;
    }
    
    return data;
}

inline std::optional<BatteryData> TelemetryClient::getBattery() const {
    if (!shared_memory || !shared_memory->battery_valid.load()) {
        return std::nullopt;
    }
    
    BatteryData data;
    data.voltage = shared_memory->battery.voltage;
    data.current = shared_memory->battery.current;
    data.capacity = shared_memory->battery.capacity;
    data.remaining_percent = shared_memory->battery.remaining_percent;
    data.timestamp_us = shared_memory->battery.timestamp_us;
    
    if (!isRecentTimestamp(data.timestamp_us)) {
        return std::nullopt;
    }
    
    return data;
}

inline std::optional<LinkData> TelemetryClient::getLink() const {
    if (!shared_memory || !shared_memory->link_valid.load()) {
        return std::nullopt;
    }
    
    LinkData data;
    data.uplink_rssi = shared_memory->link.uplink_rssi;
    data.downlink_rssi = shared_memory->link.downlink_rssi;
    data.link_quality = shared_memory->link.link_quality;
    data.snr = shared_memory->link.snr;
    data.timestamp_us = shared_memory->link.timestamp_us;
    
    if (!isRecentTimestamp(data.timestamp_us)) {
        return std::nullopt;
    }
    
    return data;
}

inline std::pair<uint32_t, std::string> TelemetryClient::getErrorInfo() const {
    if (!shared_memory) {
        return {0, "Not connected"};
    }
    
    return {shared_memory->error_count.load(), std::string(shared_memory->last_error)};
}

inline bool TelemetryClient::hasNewData() {
    if (!shared_memory) {
        return false;
    }
    
    uint32_t current_seq = shared_memory->sequence_number.load();
    if (current_seq != last_sequence_number) {
        last_sequence_number = current_seq;
        return true;
    }
    
    return false;
}

// Singleton implementation
inline TelemetryStore& TelemetryStore::getInstance() {
    static TelemetryStore instance;
    return instance;
}

inline bool TelemetryStore::connect() {
    return client.connect();
}

inline void TelemetryStore::disconnect() {
    client.disconnect();
}

inline bool TelemetryStore::isConnected() const {
    return client.isConnected();
}

inline std::optional<AttitudeData> TelemetryStore::getAttitude() const {
    return client.getAttitude();
}

inline std::optional<BatteryData> TelemetryStore::getBattery() const {
    return client.getBattery();
}

inline std::optional<LinkData> TelemetryStore::getLink() const {
    return client.getLink();
}

inline bool TelemetryStore::hasNewData() {
    return client.hasNewData();
}

} // namespace droneforge::telemetry 