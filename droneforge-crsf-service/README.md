# DroneForge CRSF Service

A high-performance CRSF (Crossfire) protocol service providing real-time telemetry data access via shared memory IPC.

## Overview

DroneForge CRSF Service is a specialized communication service that handles CRSF protocol communication with drone flight controllers and radio systems. It provides ultra-low latency telemetry data access (1-2μs) through high-performance shared memory, making it ideal for real-time applications requiring immediate access to flight data.

## Features

- **High-Performance IPC**: Sub-microsecond telemetry data access via shared memory
- **Cross-Platform**: Native support for Windows and Linux
- **Real-Time Telemetry**: Battery, attitude, link statistics, and flight mode data
- **Device Discovery**: Automatic CRSF device detection and management
- **Robust Communication**: Error recovery and connection monitoring
- **Dual Licensing**: GPL-3.0 service with MIT client interface for proprietary integration

## Architecture

The service consists of two main components:

1. **CRSF Service** (GPL-3.0): Handles CRSF protocol communication and creates shared memory
2. **Client Interface** (MIT): Header-only library for accessing telemetry data from applications

```
┌─────────────────┐    CRSF     ┌──────────────────┐    Shared    ┌─────────────────┐
│ Flight          │◄──────────►│ DroneForge CRSF  │    Memory    │ Client          │
│ Controller      │   Protocol  │ Service          │◄────────────►│ Applications    │
└─────────────────┘             └──────────────────┘              └─────────────────┘
```

## Requirements

### Build Requirements
- C++17 compatible compiler (GCC 8+, Clang 7+, MSVC 2019+)
- CMake 3.16 or later
- fmt library

### Runtime Requirements
- Serial port access permissions
- Windows: COM port access
- Linux: Access to `/dev/ttyUSB*` or `/dev/ttyACM*` devices

## Building

### Clone the Repository
```bash
git clone https://github.com/your-org/droneforge-crsf-service.git
cd droneforge-crsf-service
```

### Linux/macOS
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Windows (Visual Studio)
```cmd
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019"
cmake --build . --config Release
```

## Usage

### Running the Service

The service requires a serial port and baud rate as command line arguments:

```bash
# Linux
./droneforge-crsf-service /dev/ttyUSB0 420000

# Windows
droneforge-crsf-service.exe COM3 420000
```

**Common Baud Rates:**
- 420000 (ExpressLRS/ELRS)
- 400000 (ELRS alternative)
- 115200 (Standard CRSF)

### Service Output
```
DroneForge CRSF Service v1.0 (GPL-3.0)
High-performance telemetry service using shared memory IPC
DroneForge CRSF Service initialized
Port: /dev/ttyUSB0, Baud: 420000
CRSF Service running... Press Ctrl+C to stop
```

## Client Integration

### Basic Usage

Include the client header in your application:

```cpp
#include "client/telemetry_client.h"
using namespace droneforge::telemetry;

int main() {
    TelemetryClient client;
    
    if (!client.connect()) {
        std::cerr << "Failed to connect to CRSF service" << std::endl;
        return 1;
    }
    
    while (client.isConnected()) {
        // Get attitude data
        auto attitude = client.getAttitude();
        if (attitude) {
            std::cout << "Pitch: " << attitude->pitch << "°, "
                      << "Roll: " << attitude->roll << "°, "
                      << "Yaw: " << attitude->yaw << "°" << std::endl;
        }
        
        // Get battery data
        auto battery = client.getBattery();
        if (battery) {
            std::cout << "Voltage: " << battery->voltage << "V, "
                      << "Current: " << battery->current << "A, "
                      << "Remaining: " << (int)battery->remaining_percent << "%" << std::endl;
        }
        
        // Get link statistics
        auto link = client.getLink();
        if (link) {
            std::cout << "RSSI: " << link->uplink_rssi << "dBm, "
                      << "LQ: " << (int)link->link_quality << "%" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
```

### Singleton Pattern

For global access throughout your application:

```cpp
#include "client/telemetry_client.h"
using namespace droneforge::telemetry;

// Initialize once
TelemetryStore::getInstance().connect();

// Use anywhere in your application
auto attitude = TelemetryStore::getInstance().getAttitude();
if (attitude) {
    // Process attitude data
}
```

### Data Freshness

All telemetry data includes timestamps and automatic freshness validation:

```cpp
auto battery = client.getBattery();
if (battery) {
    auto age_ms = (getCurrentTimestamp() - battery->timestamp_us) / 1000;
    if (age_ms < 100) {  // Data is fresh (< 100ms old)
        // Use battery data
    }
}
```

## API Reference

### TelemetryClient Class

#### Methods
- `bool connect()` - Connect to shared memory
- `void disconnect()` - Disconnect from shared memory  
- `bool isConnected()` - Check connection status
- `std::optional<AttitudeData> getAttitude()` - Get attitude data
- `std::optional<BatteryData> getBattery()` - Get battery data
- `std::optional<LinkData> getLink()` - Get link statistics
- `bool hasNewData()` - Check for new data since last call

### Data Structures

#### AttitudeData
```cpp
struct AttitudeData {
    float pitch;           // degrees
    float roll;            // degrees  
    float yaw;             // degrees
    uint64_t timestamp_us; // microseconds since epoch
};
```

#### BatteryData
```cpp
struct BatteryData {
    float voltage;                // volts
    float current;                // amps
    float capacity;               // mAh
    uint8_t remaining_percent;    // %
    uint64_t timestamp_us;        // microseconds since epoch
};
```

#### LinkData
```cpp
struct LinkData {
    float uplink_rssi;       // dBm
    float downlink_rssi;     // dBm
    uint8_t link_quality;    // %  
    int8_t snr;              // dB
    uint64_t timestamp_us;   // microseconds since epoch
};
```

## Troubleshooting

### Common Issues

**Service fails to start:**
- Check serial port permissions: `sudo usermod -a -G dialout $USER` (Linux)
- Verify device path: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- Ensure baud rate matches your CRSF device configuration

**No telemetry data:**
- Verify CRSF device is transmitting telemetry
- Check flight controller CRSF telemetry settings
- Monitor service output for CRSF frame reception

**Client connection fails:**
- Ensure service is running before starting client
- Check shared memory permissions (Linux: `/dev/shm/`)
- Verify no conflicting processes are using the shared memory

### Debug Mode

For detailed debugging, modify the source code to add additional logging as needed and rebuild the service.

### Performance Monitoring

The service provides performance metrics through callbacks:
```cpp
callbacks.onPerformanceMetrics = [](const std::string& threadName, float processingTimeMs) {
    std::cout << threadName << " thread: " << processingTimeMs << "ms" << std::endl;
};
```

## Contributing

We welcome contributions to improve the DroneForge CRSF Service. Please follow these guidelines:

### Development Setup
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Follow the existing code style and conventions
4. Add tests for new functionality
5. Submit a pull request with detailed description

### Code Style
- Use C++17 features where appropriate
- Follow RAII principles for resource management
- Prefer standard library over custom implementations
- Document public APIs with Doxygen comments

### Testing
- Test on both Windows and Linux platforms
- Verify with multiple CRSF devices when possible
- Include performance regression testing

## License

This project uses dual licensing:

- **Service Code** (src/main.cpp, src/crsf/, src/serial/): [GPL-3.0](LICENSE-GPL)
- **Client Interface** (src/client/): [MIT](LICENSE-MIT)

The dual licensing allows:
- Open source applications to use the complete service under GPL-3.0
- Proprietary applications to integrate the client interface under MIT

## Acknowledgments

- Based on CRSF protocol specifications from ExpressLRS project
- Serial communication adapted from wjwwood/serial library
- Cross-platform compatibility helpers from various open source projects

## Support

For bug reports and feature requests, please use the GitHub Issues tracker.

For commercial support and integration assistance, contact DroneForge support. 