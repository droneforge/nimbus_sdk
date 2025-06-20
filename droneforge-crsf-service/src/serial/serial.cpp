/* SPDX-License-Identifier: MIT
 * Copyright © Craig Lilley
 * Copyright © Yan Naing Aye
 * Copyright © Mark Collins
 * Heavily derivative of wjwwood/serial, see https://github.com/wjwwood/serial/tree/69e0372cf0d3796e84ce9a09aff1d74496f68720/src/impl/list_ports
 * Heavily derivative of yan9a/serial, see https://github.com/yan9a/serial/blob/7346457660b192a38660f53e0ea11565719682f4/ceSerial.h
 */

#include "serial.h"

#include <thread>  // Added for std::this_thread

#if SERIAL_WIN
    #include <devguid.h>
    #include <initguid.h>

    #include <setupapi.h>
    #include <tchar.h>
#elif SERIAL_LINUX
    #include <filesystem>
    #include <fstream>
    #include <regex>
    #include <glob.h>

    #include <errno.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>

    #if defined(__linux__)
        #include <linux/serial.h>
        #include <linux/tty_flags.h>
    #elif defined(__APPLE__)
        #include <sys/types.h>
    #endif

    #include <sys/ioctl.h>
    #include <sys/select.h>
    #include <sys/stat.h>
#else
    #error "Unsupported platform"
#endif

#include <common/error.h>
#include <common/scope.h>

std::vector<SerialPortInfo> ListSerialPorts() {
    std::vector<SerialPortInfo> ports;

#if SERIAL_WIN

    HDEVINFO deviceInfoSet{SetupDiGetClassDevs((const GUID*)&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT)};
    if (deviceInfoSet == INVALID_HANDLE_VALUE)
        return ports;
    SCOPE_EXIT(SetupDiDestroyDeviceInfoList(deviceInfoSet));

    SP_DEVINFO_DATA deviceInfoData{.cbSize = sizeof(SP_DEVINFO_DATA)};

    for (DWORD deviceInfoSetIndex{}; SetupDiEnumDeviceInfo(deviceInfoSet, deviceInfoSetIndex, &deviceInfoData); deviceInfoSetIndex++) {
        // Get device path.
        std::string devicePath;
        {
            HKEY hkey{SetupDiOpenDevRegKey(
                deviceInfoSet,
                &deviceInfoData,
                DICS_FLAG_GLOBAL,
                0,
                DIREG_DEV,
                KEY_READ)};
            SCOPE_EXIT(RegCloseKey(hkey));

            DWORD devicePathLength{};
            LONG returnCode{RegQueryValueExA(
                hkey,
                "PortName",
                NULL,
                NULL,
                NULL,
                &devicePathLength)};

            if (returnCode != EXIT_SUCCESS || devicePathLength == 0)
                continue;

            devicePath.resize(devicePathLength - 1); // String length includes null terminator.

            returnCode = RegQueryValueExA(
                hkey,
                "PortName",
                NULL,
                NULL,
                (LPBYTE)devicePath.data(),
                &devicePathLength);

            if (returnCode != EXIT_SUCCESS)
                continue;
        }

        // Ignore parallel ports.
        if (devicePath.find("LPT") != std::string::npos)
            continue;

        // Get friendly name and hardware ID.
        auto getDeviceRegistryString = [&](DWORD property) -> std::string {
            DWORD length{};
            SetupDiGetDeviceRegistryProperty(
                deviceInfoSet,
                &deviceInfoData,
                property,
                NULL,
                NULL,
                0,
                &length);

            if (length == 0)
                return {};

            std::string value;
            value.resize(length);

            SetupDiGetDeviceRegistryProperty(
                deviceInfoSet,
                &deviceInfoData,
                property,
                NULL,
                (PBYTE)value.data(),
                length,
                &length);

            return value;
        };

        ports.emplace_back(SerialPortInfo{
            devicePath,
            getDeviceRegistryString(SPDRP_FRIENDLYNAME),
            getDeviceRegistryString(SPDRP_HARDWAREID),
        });
    }

#elif SERIAL_LINUX

    const std::string sysClassTTY{"/sys/class/tty/"};
    const std::regex ttyPattern{"^ttyS|^ttyUSB|^ttyACM"};

    // Try using /sys/class/tty first (Linux style), with fallback for macOS
    try {
        if (std::filesystem::exists(sysClassTTY)) {
            for (const auto& entry : std::filesystem::directory_iterator(sysClassTTY)) {
#ifdef __linux__
                // Linux-specific: Additional safety checks
                if (!entry.exists() || !entry.is_directory()) {
                    continue;
                }
#endif
                std::string portName{entry.path().filename().string()};
                if (!std::regex_search(portName, ttyPattern))
                    continue;

                std::string devicePath{"/dev/" + portName};
                std::string friendlyName;
                std::string hardwareID;

#ifdef __linux__
                // Linux-specific: Check device accessibility before adding
                if (access(devicePath.c_str(), F_OK) != 0) {
                    continue; // Device file doesn't exist or isn't accessible
                }
#endif

                std::ifstream ueventFile(entry.path() / "device" / "uevent");
                if (ueventFile) {
                    std::string line;
                    while (std::getline(ueventFile, line)) {
                        if (line.find("PRODUCT=") != std::string::npos)
                            hardwareID = line.substr(line.find('=') + 1);
                        if (line.find("DRIVER=") != std::string::npos)
                            friendlyName = line.substr(line.find('=') + 1);
                    }
                }

                // Order by ttyUSB, ttyACM, ttyS. Since it's far more likely for the target device to be USB.
                auto it{ports.begin()};
                bool isUSB{portName.find("ttyUSB") != std::string::npos}, isACM{portName.find("ttyACM") != std::string::npos};
                for (; it != ports.end(); it++) {
                    bool isUSB2{it->devicePath.find("ttyUSB") != std::string::npos}, isACM2{it->devicePath.find("ttyACM") != std::string::npos};
                    if (isUSB && !isUSB2)
                        break;
                    if (isACM && !isACM2)
                        break;
                }

                ports.emplace(it, SerialPortInfo{
                                      devicePath,
                                      friendlyName.empty() ? portName : friendlyName + " (" + portName + ")",
                                      hardwareID,
                                  });
            }
        }
    } catch (const std::filesystem::filesystem_error&) {
        // On some systems like macOS, /sys/class/tty may not exist. Fall back to globbing /dev.
    }

#if defined(__APPLE__)
    // macOS fallback: look for /dev/tty.* and /dev/cu.*
    glob_t glob_result{};
    if (glob("/dev/tty.*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            std::string devicePath{glob_result.gl_pathv[i]};
            ports.emplace_back(SerialPortInfo{devicePath, std::filesystem::path(devicePath).filename().string(), ""});
        }
        globfree(&glob_result);
    }

    if (glob("/dev/cu.*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            std::string devicePath{glob_result.gl_pathv[i]};
            ports.emplace_back(SerialPortInfo{devicePath, std::filesystem::path(devicePath).filename().string(), ""});
        }
        globfree(&glob_result);
    }
#endif

#endif

    return ports;
}

#if SERIAL_WIN
constexpr COMMTIMEOUTS DefaultTimeouts{
    .ReadIntervalTimeout = 0,
    .ReadTotalTimeoutMultiplier = 0,
    .ReadTotalTimeoutConstant = 0,
    .WriteTotalTimeoutMultiplier = 0,
    .WriteTotalTimeoutConstant = 0,
};
#endif

SerialPort::SerialPort(std::string devicePath, size_t baudRate) {
#if SERIAL_WIN

    handle = CreateFileA(  // Using CreateFileA for ANSI strings
        devicePath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (handle == INVALID_HANDLE_VALUE)
        throw Exception("Failed to open serial port ({}): {}", devicePath, GetLastError());

    DCB dcbSerialParams{.DCBlength = sizeof(dcbSerialParams)};

    if (!GetCommState(handle, &dcbSerialParams))
        throw Exception("Failed to get serial port state ({}): {}", devicePath, GetLastError());

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.StopBits = ONESTOPBIT;

    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fOutxDsrFlow = false;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(handle, &dcbSerialParams))
        throw Exception("Failed to set serial port state ({}): {}", devicePath, GetLastError());

    if (!SetCommTimeouts(handle, const_cast<LPCOMMTIMEOUTS>(&DefaultTimeouts)))
        throw Exception("Failed to set serial port timeouts ({}): {}", devicePath, GetLastError());

#elif SERIAL_LINUX

    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)
        throw Exception("Failed to open serial port ({}): {}", devicePath, strerror(errno));

    if (tcflush(fd, TCIOFLUSH) == -1)
        throw Exception("Failed to flush serial port ({}): {}", devicePath, strerror(errno));
    if (tcdrain(fd) == -1)
        throw Exception("Failed to drain serial port ({}): {}", devicePath, strerror(errno));
    if (tcflow(fd, TCOOFF) == -1)
        throw Exception("Failed to stop serial port ({}): {}", devicePath, strerror(errno));

    struct termios tty {
        .c_iflag = 0,
        .c_oflag = 0,
        .c_cflag = CREAD | CLOCAL | CS8,
        .c_lflag = 0,
    };
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    speed_t linuxBaudRate{[baudRate]() -> speed_t {
    #define RATE_CHECK(x)    \
        if (baudRate <= x) { \
            return B##x;     \
        }
        RATE_CHECK(50);
        RATE_CHECK(75);
        RATE_CHECK(110);
        RATE_CHECK(134);
        RATE_CHECK(150);
        RATE_CHECK(200);
        RATE_CHECK(300);
        RATE_CHECK(600);
        RATE_CHECK(1200);
        RATE_CHECK(1800);
        RATE_CHECK(2400);
        RATE_CHECK(4800);
        RATE_CHECK(9600);
        RATE_CHECK(19200);
        RATE_CHECK(38400);
        RATE_CHECK(57600);
        RATE_CHECK(115200);
        RATE_CHECK(230400);
#if defined(__linux__)
        // Linux-specific baud rates
        RATE_CHECK(460800);
        RATE_CHECK(500000);
        RATE_CHECK(576000);
        RATE_CHECK(921600);
        RATE_CHECK(1000000);
        RATE_CHECK(1152000);
        RATE_CHECK(1500000);
        RATE_CHECK(2000000);
        RATE_CHECK(2500000);
        RATE_CHECK(3000000);
        RATE_CHECK(3500000);
        return B4000000;
#elif defined(__APPLE__)
        // macOS has a more limited set of baud rates
        return B230400; // Use the highest standard baud rate available on macOS
#endif
    #undef RATE_CHECK
    }()};
    if (cfsetospeed(&tty, linuxBaudRate) == -1)
        throw Exception("Failed to set serial port output speed ({}): {}", devicePath, strerror(errno));
    if (cfsetispeed(&tty, linuxBaudRate) == -1)
        throw Exception("Failed to set serial port input speed ({}): {}", devicePath, strerror(errno));

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        throw Exception("Failed to set serial port state ({}): {}", devicePath, strerror(errno));

    if (tcflow(fd, TCOON) == -1)
        throw Exception("Failed to start serial port ({}): {}", devicePath, strerror(errno));

#endif
}

SerialPort::~SerialPort() {
#if SERIAL_WIN

    if (handle != INVALID_HANDLE_VALUE)
        CloseHandle(handle);

#elif SERIAL_LINUX

    if (fd != -1) {
        tcdrain(fd);
        close(fd);
        fd = -1;
    }

#endif
}

bool SerialPort::IsOpen() {
#if SERIAL_WIN

    DWORD errors{};
    COMSTAT comStat{};
    if (!ClearCommError(handle, &errors, &comStat) || (errors & (CE_OVERRUN | CE_RXOVER | CE_RXPARITY | CE_FRAME | CE_BREAK)))
        return false;

    return true;

#elif SERIAL_LINUX

    struct stat st {};
    if (fstat(fd, &st) == -1 || st.st_nlink == 0)
        /* st_nlink is the number of hard links to the file, these are only 0 when the file no longer exists.
         * This happens when the serial device has been disconnected.
         */
        return false;

    return true;

#endif
}

#if SERIAL_WIN
struct RaiiHandle {
    ~RaiiHandle() {
        if (handle != INVALID_HANDLE_VALUE)
            CloseHandle(handle);
    }

    HANDLE handle{INVALID_HANDLE_VALUE};
};

static thread_local RaiiHandle OverlapEvent{};

static HANDLE GetOverlapEvent() {
    if (OverlapEvent.handle == INVALID_HANDLE_VALUE) {
        OverlapEvent.handle = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (!OverlapEvent.handle)
            throw Exception("Failed to create overlapped event: {}", GetLastError());
    }
    return OverlapEvent.handle;
}

size_t ReadSerialFile(HANDLE handle, void* buffer, size_t bufferSize, std::chrono::time_point<std::chrono::steady_clock> timeout) {
    OVERLAPPED overlapped{
        .hEvent = GetOverlapEvent(),
    };

    if (!ReadFile(handle, buffer, bufferSize, nullptr, &overlapped)) {
        if (GetLastError() != ERROR_IO_PENDING)
            throw Exception("Failed to read from serial port: {}", GetLastError());
    }

    DWORD timeoutDword{INFINITE};
    if (timeout != std::chrono::time_point<std::chrono::steady_clock>{}) {
        auto timeoutMs{std::chrono::duration_cast<std::chrono::milliseconds>(timeout - std::chrono::steady_clock::now()).count()};
        timeoutDword = timeoutMs <= 0 ? 0 : static_cast<DWORD>(timeoutMs);
    }

    bool cancelled{};
    if (WaitForSingleObject(overlapped.hEvent, timeoutDword) != WAIT_OBJECT_0) {
        if (!CancelIoEx(handle, &overlapped))
            throw Exception("Failed to cancel I/O: {}", GetLastError());
        cancelled = true;
    }

    DWORD bytesRead{};
    int retryCount = 0;
    const int maxRetries = 3;
    while (retryCount < maxRetries) {
        if (GetOverlappedResult(handle, &overlapped, &bytesRead, TRUE)) {
            break;
        }
        
        DWORD error = GetLastError();
        if (error == ERROR_PIPE_BUSY || error == ERROR_OPERATION_ABORTED) {
            ::std::this_thread::sleep_for(::std::chrono::milliseconds(10 * (retryCount + 1)));
            retryCount++;
            continue;
        }
        
        if (cancelled)
            return 0;
        throw Exception("Failed to get overlapped result: {}", error);
    }

    return bytesRead;
}

constexpr DWORD MaxTimeout{std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::hours(1)).count()};
#endif

std::optional<u8> SerialPort::ReadByte(std::chrono::time_point<std::chrono::steady_clock> timeout) {
    if (std::chrono::steady_clock::now() >= timeout)
        return std::nullopt;

    u8 byte{};
#if SERIAL_WIN

    size_t bytesRead{ReadSerialFile(handle, &byte, 1, timeout)};
    if (bytesRead == 0)
        return std::nullopt;

#elif SERIAL_LINUX

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    ssize_t bytesRead{};
    do {
        if (std::chrono::steady_clock::now() >= timeout)
            return std::nullopt;

        constexpr int UsInSecond{1000000};
        struct timeval tv {
            .tv_sec = std::chrono::duration_cast<std::chrono::seconds>(timeout - std::chrono::steady_clock::now()).count(),
#ifdef __APPLE__
            .tv_usec = static_cast<__darwin_suseconds_t>(std::chrono::duration_cast<std::chrono::microseconds>(timeout - std::chrono::steady_clock::now()).count() % UsInSecond),
#else
            .tv_usec = static_cast<suseconds_t>(std::chrono::duration_cast<std::chrono::microseconds>(timeout - std::chrono::steady_clock::now()).count() % UsInSecond),
#endif
        };

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        if (select(fd + 1, &readfds, NULL, NULL, &tv) == -1)
            throw Exception("Failed to wait for serial port: {}", strerror(errno));

        if (!FD_ISSET(fd, &readfds))
            return std::nullopt;

        bytesRead = read(fd, &byte, 1);
        if (bytesRead == -1)
            throw Exception("Failed to read from serial port: {}", strerror(errno));
    } while (bytesRead == 0);

    if (bytesRead != 1)
        throw Exception("Failed to read from serial port: expected 1, got {}", bytesRead);

#endif

    return byte;
}

void SerialPort::Read(compat::span<u8> buffer) {
#if SERIAL_WIN

    size_t bytesRead{ReadSerialFile(handle, buffer.data(), buffer.size(), std::chrono::steady_clock::now() + std::chrono::milliseconds(MaxTimeout))};
    if (bytesRead != buffer.size())
        throw Exception("Failed to read from serial port: expected {}, got {}", buffer.size(), bytesRead);

#elif SERIAL_LINUX

    while (buffer.size() > 0) {
        ssize_t bytesRead{read(fd, buffer.data(), buffer.size())};
        if (bytesRead == -1)
            throw Exception("Failed to read from serial port: {}", strerror(errno));
        buffer = buffer.subspan(bytesRead);
    }

#endif
}

void SerialPort::Write(compat::span<u8> buffer) {
#if SERIAL_WIN

    OVERLAPPED overlapped{
        .hEvent = GetOverlapEvent(),
    };

    if (!WriteFile(handle, buffer.data(), buffer.size(), nullptr, &overlapped)) {
        if (GetLastError() != ERROR_IO_PENDING)
            throw Exception("Failed to write to serial port: {}", GetLastError());
    }

    DWORD bytesWritten{};
    int retryCount = 0;
    const int maxRetries = 3;
    while (retryCount < maxRetries) {
        if (GetOverlappedResult(handle, &overlapped, &bytesWritten, TRUE)) {
            break;
        }
        
        DWORD error = GetLastError();
        if (error == ERROR_PIPE_BUSY || error == ERROR_OPERATION_ABORTED) {
            ::std::this_thread::sleep_for(::std::chrono::milliseconds(10 * (retryCount + 1)));
            retryCount++;
            continue;
        }
        
        throw Exception("Failed to get overlapped result: {}", error);
    }

    if (bytesWritten != buffer.size())
        throw Exception("Failed to write to serial port: expected {}, got {}", buffer.size(), bytesWritten);

#elif SERIAL_LINUX

    while (buffer.size() > 0) {
        ssize_t bytesWritten{write(fd, buffer.data(), buffer.size())};
        if (bytesWritten == -1)
            throw Exception("Failed to write to serial port: {}", strerror(errno));
        buffer = buffer.subspan(bytesWritten);
    }

#endif
}

void SerialPort::SetDTR(bool state) {
#if SERIAL_WIN

    if (!EscapeCommFunction(handle, state ? SETDTR : CLRDTR))
        throw Exception("Failed to set DTR: {}", GetLastError());

#elif SERIAL_LINUX

    if (ioctl(fd, state ? TIOCMBIS : TIOCMBIC, TIOCM_DTR) == -1)
        throw Exception("Failed to set DTR: {}", strerror(errno));
#endif
}

void SerialPort::SetRTS(bool state) {
#if SERIAL_WIN

    if (!EscapeCommFunction(handle, state ? SETRTS : CLRRTS))
        throw Exception("Failed to set RTS: {}", GetLastError());

#elif SERIAL_LINUX

    if (ioctl(fd, state ? TIOCMBIS : TIOCMBIC, TIOCM_RTS) == -1)
        throw Exception("Failed to set RTS: {}", strerror(errno));

#endif
}

void SerialPort::Open(std::string devicePath, u32 baudRate) {
    Close();

#if SERIAL_WIN
    handle = CreateFileA(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        throw Exception("Failed to open serial port ({}): {}", devicePath, GetLastError());

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb))
        throw Exception("Failed to get serial port state ({}): {}", devicePath, GetLastError());

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    
    // Enable hardware flow control if available
    // Note: Some USB-to-Serial adapters may not support hardware flow control
    dcb.fOutxCtsFlow = TRUE;             // Enable CTS flow control for output
    dcb.fRtsControl = RTS_CONTROL_TOGGLE; // Toggle RTS when transmitting

    if (!SetCommState(handle, &dcb))
        throw Exception("Failed to set serial port state ({}): {}", devicePath, GetLastError());

    // Set larger buffer sizes to handle telemetry data bursts
    if (!SetupComm(handle, 4096, 4096)) // Increase from default values
        fmt::print("Warning: Failed to increase comm buffer sizes: {}\n", GetLastError());

    // Set timeouts for more robust operation with telemetry
    COMMTIMEOUTS timeouts{};
    // Read operations complete immediately with bytes that are already available
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    // Write operations have a reasonable timeout
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 500;
    
    if (!SetCommTimeouts(handle, &timeouts))
        throw Exception("Failed to set serial port timeouts ({}): {}", devicePath, GetLastError());

    // Purge any remaining data
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

#elif SERIAL_LINUX

// ... existing Linux code ...

#endif
}

void SerialPort::Close() {
#if SERIAL_WIN
    if (handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
        handle = INVALID_HANDLE_VALUE;
    }
#elif SERIAL_LINUX
    if (fd != -1) {
        tcdrain(fd);
        close(fd);
        fd = -1;
    }
#endif
}