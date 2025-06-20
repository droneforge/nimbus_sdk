/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#pragma once

#include <list>

#include "frame.h"
#include "parameter.h"
#include "protocol.h"

#include "../serial/serial.h"

namespace crsf {

/**
 * @brief Main interface for CRSF (Crossfire) protocol communication.
 * 
 * This class provides a high-level interface for communicating with CRSF devices,
 * handling frame transmission, reception, and device management. It implements
 * the full CRSF protocol specification including telemetry, parameter updates,
 * and device discovery.
 */
class Interface {
  private:
    SerialPort serial;
    FrameData oldFrame{}; //!< A copy of the previous frame, used to prevent duplicate frames from being processed.

  public:
    /**
     * @brief Constructs a new CRSF interface.
     * 
     * @param port Serial port name/path
     * @param baud Baud rate for serial communication
     * @throws SerialException if port cannot be opened
     */
    Interface(std::string port, size_t baud);

    /**
     * @brief Checks if the interface is active.
     * 
     * @return true if the serial port is open and operational
     * @return false if the connection is closed or has errors
     */
    bool IsActive() noexcept;

    /**
     * @brief Attempts to reset the connected device.
     * 
     * Resets the device by toggling DTR and RTS lines. This is useful when
     * the device becomes unresponsive or needs to be reinitialized.
     */
    void ResetDevice() noexcept;

    /**
     * @brief Sends a CRSF frame to the device.
     * 
     * @param frame The frame to send
     * @throws SerialException if transmission fails
     */
    void SendFrame(const FrameV& frame);

    /**
     * @brief Receives a CRSF frame from the device.
     * 
     * This function implements the CRSF frame reception protocol, including:
     * - Sync byte detection
     * - Frame validation
     * - CRC checking
     * - Duplicate frame filtering
     * 
     * @param timeout Maximum time to wait for a frame
     * @return std::optional<FrameV> The received frame, or nullopt if no frame received
     * @throws Exception if frame validation fails
     */
    std::optional<FrameV> ReceiveFrame(std::chrono::time_point<std::chrono::steady_clock> timeout);
};

/**
 * @brief Represents a CRSF device with its parameters and state.
 * 
 * This class maintains the state of a CRSF device including its parameters,
 * metadata, and pending operations. It handles parameter updates and
 * provides an interface for device configuration.
 */
struct Device {
    Address address;
    std::string name;
    DeviceMetadata metadata;
    std::vector<Parameter> parameters;
    std::list<ParameterWriteFrame> parameterWrites;
    std::optional<OpenTxSyncData> syncData;

    /**
     * @brief Constructs a new Device object.
     * 
     * @param address Device address
     * @param name Device name
     * @param metadata Device metadata
     */
    Device(Address address, const std::string& name, DeviceMetadata metadata);

    /**
     * @brief Updates device metadata and resets state if necessary.
     * 
     * @param name New device name
     * @param metadata New device metadata
     */
    void Update(const std::string& name, DeviceMetadata metadata);

    /**
     * @brief Finds a parameter by its field index.
     * 
     * @param fieldIndex Parameter field index
     * @return Parameter* Pointer to the parameter, or nullptr if not found
     */
    Parameter* FindParameter(u8 fieldIndex);

    /**
     * @brief Writes a value to a device parameter.
     * 
     * This function handles parameter updates, including:
     * - Value validation
     * - Dependent parameter reset
     * - Command parameter handling
     * 
     * @param fieldIndex Parameter field index
     * @param value New parameter value
     */
    void WriteParameter(u8 fieldIndex, ParameterWriteFrame::WriteValue value);

    /**
     * @brief Gets the next frame to transmit.
     * 
     * @return std::optional<FrameV> Next frame to send, or nullopt if none pending
     */
    std::optional<FrameV> TryGetTxFrame();
};
} // namespace crsf