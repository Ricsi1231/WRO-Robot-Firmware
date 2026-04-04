/**
 * @file IPeripheryBus.hpp
 * @brief Hardware-agnostic interface for periphery bus communication.
 *
 * Defines a generic bus API that works with any low-level serial transport
 * (SPI, I2C, UART, etc.). Concrete implementations inherit from this
 * interface and provide device-specific functionality.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace WroRobotSoftware {
namespace PeripheryBus {

/**
 * @enum TypeId
 * @brief Identifies the physical bus type.
 */
enum class TypeId : uint8_t {
    UNDEFINED = 0,  ///< Not configured
    SPI,            ///< SPI bus
    I2C,            ///< I2C bus
    UART,           ///< UART bus
};

/**
 * @enum BusStatus
 * @brief Return status codes for bus operations.
 */
enum class BusStatus : uint8_t {
    OK = 0,            ///< Operation completed successfully
    BUSY,              ///< Bus is currently busy
    TIMEOUT,           ///< Operation timed out
    INVALID_ARGUMENT,  ///< Invalid parameter supplied
    NOT_INITIALIZED,   ///< Bus has not been initialized
    ERROR,             ///< General error
};

/**
 * @struct DeviceDescriptor
 * @brief Describes a device attached to the bus.
 */
struct DeviceDescriptor {
    uint8_t id;       ///< Device identifier (e.g. chip-select index, I2C address, or 0 for UART)
    uint32_t config;  ///< Bus-specific device configuration
};

/**
 * @class IPeripheryBus
 * @brief Abstract interface for a low-level periphery bus.
 *
 * @par Thread safety
 * Implementations must be thread-safe. All public methods that access
 * hardware or shared state must be serialized through an internal mutex.
 * The @c takeMutex parameter on transfer methods allows the caller to
 * opt out of internal locking when the caller already holds the bus
 * mutex (e.g. for multi-step atomic sequences). isBusy() is lock-free
 * and may be called from any context.
 *
 * @par Lifecycle
 * Callers must follow this ordering:
 *   1. Construct the bus object.
 *   2. Call init() exactly once.
 *   3. Call addDevice() for each device on the bus.
 *   4. Call transfer() / transferAndReceive() as needed.
 *   5. Call deinit() when finished (safe to omit if the destructor
 *      handles cleanup).
 * Calling transfer methods before init() or addDevice() returns
 * @c BusStatus::NOT_INITIALIZED.
 */
class IPeripheryBus {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IPeripheryBus() = default;

    /**
     * @brief Get the bus type identifier.
     * @return TypeId of this bus implementation.
     */
    virtual TypeId getType() const = 0;

    /**
     * @brief Initialize the bus hardware.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus init() = 0;

    /**
     * @brief De-initialize the bus hardware and release resources.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus deinit() = 0;

    /**
     * @brief Register a device on the bus.
     * @param descriptor Device descriptor with id and configuration.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus addDevice(const DeviceDescriptor& descriptor) = 0;

    /**
     * @brief Transmit data on the bus.
     * @param txData Pointer to the data buffer to send.
     * @param txSize Number of bytes to send.
     * @param takeMutex If true, acquire the bus mutex before the operation.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus transfer(const uint8_t* txData, size_t txSize, bool takeMutex = true) = 0;

    /**
     * @brief Transmit and/or receive data on the bus.
     * @param txData Pointer to the data buffer to send (may be nullptr for receive-only).
     * @param txSize Number of bytes to send.
     * @param rxData Pointer to the buffer to store received data.
     * @param rxSize [in,out] On input: maximum bytes to read (buffer capacity).
     *               On output: actual bytes received. Set to 0 on error.
     * @param takeMutex If true, acquire the bus mutex before the operation.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus transferAndReceive(const uint8_t* txData, size_t txSize, uint8_t* rxData, size_t* rxSize, bool takeMutex = true) = 0;

    /**
     * @brief Check if the bus is currently busy with an operation.
     * @return true if busy, false otherwise.
     */
    virtual bool isBusy() const = 0;

    /**
     * @brief Abort any ongoing transfer and flush buffers.
     * @return BusStatus::OK on success, else error status.
     */
    virtual BusStatus abort() = 0;
};

}  // namespace PeripheryBus
}  // namespace WroRobotSoftware
