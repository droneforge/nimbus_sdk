/* SPDX-License-Identifier: GPL-3.0-only
 * DroneForge CRSF Service - Main Service Executable
 * 
 * High-performance CRSF communication service with shared memory telemetry interface.
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <signal.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#include "crsf/crsf_manager.hpp"

// Shared memory structure for telemetry data
struct TelemetrySharedMemory {
    std::atomic<uint32_t> sequence_number{0};
    std::atomic<bool> attitude_valid{false};
    std::atomic<bool> battery_valid{false};
    std::atomic<bool> link_valid{false};
    
    // Attitude data
    struct {
        float pitch;
        float roll; 
        float yaw;
        uint64_t timestamp_us;
    } attitude;
    
    // Battery data
    struct {
        float voltage;
        float current;
        float capacity;
        uint8_t remaining_percent;
        uint64_t timestamp_us;
    } battery;
    
    // Link statistics
    struct {
        float uplink_rssi;
        float downlink_rssi;
        uint8_t link_quality;
        int8_t snr;
        uint64_t timestamp_us;
    } link;
    
    // Status
    std::atomic<bool> service_running{true};
    std::atomic<uint32_t> error_count{0};
    char last_error[256] = {};
};

class CrsfService {
private:
    std::unique_ptr<droneforge::communication::CrsfManager> crsf_manager;
    TelemetrySharedMemory* shared_memory = nullptr;
    std::atomic<bool> should_stop{false};
    
#ifdef _WIN32
    HANDLE shm_handle = INVALID_HANDLE_VALUE;
#else
    int shm_fd = -1;
#endif

public:
    CrsfService(const std::string& port, size_t baud_rate) {
        try {
            // Create shared memory
            CreateSharedMemory();
            
            // Initialize CRSF manager
            crsf_manager = std::make_unique<droneforge::communication::CrsfManager>(port, baud_rate);
            
            // Set up callbacks
            droneforge::communication::CrsfEventCallbacks callbacks;
            callbacks.onAttitude = [this](const crsf::AttitudeData& data) {
                OnAttitudeData(data);
            };
            callbacks.onBattery = [this](const crsf::BatterySensorData& data) {
                OnBatteryData(data);
            };
            callbacks.onLinkStats = [this](const crsf::LinkStatisticsData& data) {
                OnLinkData(data);
            };
            callbacks.onError = [this](const std::string& error) {
                OnError(error);
            };
            callbacks.onConnectionStatus = [this](bool connected) {
                std::cout << "CRSF Connection: " << (connected ? "Connected" : "Disconnected") << std::endl;
            };
            
            crsf_manager->setEventCallbacks(callbacks);
            
            std::cout << "DroneForge CRSF Service initialized" << std::endl;
            std::cout << "Port: " << port << ", Baud: " << baud_rate << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize CRSF service: " << e.what() << std::endl;
            throw;
        }
    }
    
    ~CrsfService() {
        if (shared_memory) {
            shared_memory->service_running = false;
        }
        
        CleanupSharedMemory();
    }
    
    void Run() {
        // Start CRSF communication threads
        crsf_manager->startThreads();
        
        std::cout << "CRSF Service running... Press Ctrl+C to stop" << std::endl;
        
        // Main service loop
        while (!should_stop && !crsf_manager->isStopRequested()) {
            // Update sequence number for clients to detect new data
            shared_memory->sequence_number++;
            
            // Sleep briefly to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "CRSF Service shutting down..." << std::endl;
    }
    
    void Stop() {
        should_stop = true;
        if (crsf_manager) {
            crsf_manager->requestStop();
        }
    }

private:
    void CreateSharedMemory() {
        const char* shm_name = "droneforge_crsf_telemetry";
        
#ifdef _WIN32
        shm_handle = CreateFileMappingA(
            INVALID_HANDLE_VALUE,
            nullptr,
            PAGE_READWRITE,
            0,
            sizeof(TelemetrySharedMemory),
            shm_name
        );
        
        if (shm_handle == nullptr) {
            throw std::runtime_error("Failed to create shared memory mapping");
        }
        
        shared_memory = static_cast<TelemetrySharedMemory*>(
            MapViewOfFile(shm_handle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(TelemetrySharedMemory))
        );
        
        if (shared_memory == nullptr) {
            CloseHandle(shm_handle);
            throw std::runtime_error("Failed to map shared memory view");
        }
#else
        shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            throw std::runtime_error("Failed to create shared memory object");
        }
        
        if (ftruncate(shm_fd, sizeof(TelemetrySharedMemory)) == -1) {
            close(shm_fd);
            shm_unlink(shm_name);
            throw std::runtime_error("Failed to set shared memory size");
        }
        
        shared_memory = static_cast<TelemetrySharedMemory*>(
            mmap(nullptr, sizeof(TelemetrySharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
        );
        
        if (shared_memory == MAP_FAILED) {
            close(shm_fd);
            shm_unlink(shm_name);
            throw std::runtime_error("Failed to map shared memory");
        }
#endif
        
        // Initialize shared memory
        new (shared_memory) TelemetrySharedMemory();
    }
    
    void CleanupSharedMemory() {
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
            shm_unlink("droneforge_crsf_telemetry");
            shm_fd = -1;
        }
#endif
    }
    
    uint64_t GetTimestampMicros() {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()
        ).count();
    }
    
    void OnAttitudeData(const crsf::AttitudeData& data) {
        shared_memory->attitude.pitch = data.Pitch();
        shared_memory->attitude.roll = data.Roll();
        shared_memory->attitude.yaw = data.Yaw();
        shared_memory->attitude.timestamp_us = GetTimestampMicros();
        shared_memory->attitude_valid = true;
    }
    
    void OnBatteryData(const crsf::BatterySensorData& data) {
        shared_memory->battery.voltage = data.Voltage();
        shared_memory->battery.current = data.Current();
        shared_memory->battery.capacity = data.Capacity();
        shared_memory->battery.remaining_percent = data.RemainingCapacity();
        shared_memory->battery.timestamp_us = GetTimestampMicros();
        shared_memory->battery_valid = true;
    }
    
    void OnLinkData(const crsf::LinkStatisticsData& data) {
        shared_memory->link.uplink_rssi = data.UplinkRssi1();
        shared_memory->link.downlink_rssi = data.DownlinkRssi();
        shared_memory->link.link_quality = data.uplinkLinkQuality;
        shared_memory->link.snr = data.uplinkSnr;
        shared_memory->link.timestamp_us = GetTimestampMicros();
        shared_memory->link_valid = true;
    }
    
    void OnError(const std::string& error) {
        shared_memory->error_count++;
        strncpy(shared_memory->last_error, error.c_str(), sizeof(shared_memory->last_error) - 1);
        shared_memory->last_error[sizeof(shared_memory->last_error) - 1] = '\0';
        
        std::cerr << "CRSF Error: " << error << std::endl;
    }
};

// Global service instance for signal handling
std::unique_ptr<CrsfService> g_service;

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_service) {
        g_service->Stop();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "DroneForge CRSF Service v1.0 (GPL-3.0)" << std::endl;
    std::cout << "High-performance telemetry service using shared memory IPC" << std::endl;
    
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <baud_rate>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /dev/ttyUSB0 420000" << std::endl;
        std::cerr << "Example: " << argv[0] << " COM3 420000" << std::endl;
        return 1;
    }
    
    std::string port = argv[1];
    size_t baud_rate = std::stoull(argv[2]);
    
    // Set up signal handlers
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    
    try {
        g_service = std::make_unique<CrsfService>(port, baud_rate);
        g_service->Run();
    } catch (const std::exception& e) {
        std::cerr << "Service error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Service stopped cleanly" << std::endl;
    return 0;
} 