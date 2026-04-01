/**
 * @file IComm.hpp
 * @brief Hardware-agnostic interface for communication peripherals.
 *
 * Defines a generic communication API that works with any serial transport
 * (USB CDC, UART, Bluetooth SPP, Wi-Fi TCP, etc.). Concrete implementations
 * inherit from this interface and provide device-specific functionality.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @class IComm
 * @brief Abstract interface for a bidirectional communication channel.
 */
class IComm {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IComm() = default;

    /**
     * @brief Initialize the communication peripheral.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Send raw binary data.
     * @param data Pointer to the data buffer to send.
     * @param dataSize Number of bytes to send.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t sendData(const uint8_t* data, size_t dataSize) = 0;

    /**
     * @brief Receive raw binary data.
     * @param data Pointer to the buffer to store received data.
     * @param rxBufferSize On input: buffer capacity; on output: actual bytes received.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t receiveData(uint8_t* data, size_t* rxBufferSize) = 0;

    /**
     * @brief Send a null-terminated string.
     * @param str Null-terminated string to send.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t sendString(const char* str) = 0;

    /**
     * @brief Check if the communication channel is connected and ready.
     * @return true if connected, false otherwise.
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief Check if new data has been received and is available for reading.
     * @return true if new data is available, false otherwise.
     */
    virtual bool newDataIsReceived() const = 0;

    /**
     * @brief Flush (discard) the receive buffer contents.
     */
    virtual void flushRxBuffer() = 0;
};

}  // namespace WroRobotSoftware
