/**
 * @file IMotorDriver.hpp
 * @brief Hardware-agnostic interface for DC motor drivers.
 *
 * Defines a generic motor driver API that works with any H-bridge driver
 * (DRV8876, L298N, TB6612, etc.). Concrete implementations inherit from
 * this interface and provide device-specific functionality.
 */

#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @enum MotorDirection
 * @brief Logical motor rotation direction.
 */
enum class MotorDirection : bool {
    LEFT = false,  ///< Counter-clockwise direction
    RIGHT = true   ///< Clockwise direction
};

/**
 * @class IMotorDriver
 * @brief Abstract interface for controlling a DC motor driver.
 */
class IMotorDriver {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IMotorDriver() = default;

    /**
     * @brief Initialize the motor driver hardware.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Set the motor rotation direction.
     * @param direction Desired motor direction.
     */
    virtual void setDirection(MotorDirection direction) = 0;

    /**
     * @brief Reverse the current motor direction.
     */
    virtual void reverseDirection() = 0;

    /**
     * @brief Safely change direction by ramping speed down, switching, then ramping back up.
     * @param direction Desired new motor direction.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t setDirectionSafe(MotorDirection direction) = 0;

    /**
     * @brief Safely reverse direction by ramping speed down, switching, then ramping back up.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t reverseDirectionSafe() = 0;

    /**
     * @brief Set motor speed as a percentage (0-100%).
     * @param speed Target speed in percent.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if out of range.
     */
    virtual esp_err_t setSpeed(uint8_t speed) = 0;

    /**
     * @brief Set motor speed with a timed ramp transition.
     * @param targetPercent Target speed in percent.
     * @param rampTimeMs Duration of the ramp in milliseconds.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t setSpeed(uint8_t targetPercent, uint32_t rampTimeMs) = 0;

    /**
     * @brief Stop the motor (set duty to zero).
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Set the motor to coast mode (outputs disabled, free spin).
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t coast() = 0;

    /**
     * @brief Set the motor to brake mode (outputs shorted).
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t brake() = 0;

    /**
     * @brief Get the current motor speed.
     * @return Motor speed in percent (0-100).
     */
    virtual uint8_t getMotorSpeed() const = 0;

    /**
     * @brief Get the current motor direction.
     * @return MotorDirection enum value.
     */
    virtual MotorDirection getMotorDirection() const = 0;

    /**
     * @brief Check if the motor is currently running.
     * @return true if motor speed > 0.
     */
    virtual bool motorIsRunning() const = 0;

    /**
     * @brief Check if a fault condition is currently active.
     * @return true if fault is triggered.
     */
    virtual bool isFaultTriggered() const = 0;

    /**
     * @brief Clear the fault flag.
     */
    virtual void clearFaultFlag() = 0;

    /**
     * @brief Get and clear the fault flag atomically.
     * @return true if a fault was present (now cleared).
     */
    virtual bool getAndClearFault() = 0;

    /**
     * @brief Register a callback to be invoked when a fault is detected.
     * @param cb Callback function.
     */
    virtual void setFaultCallback(const std::function<void()>& cb) = 0;

    /**
     * @brief Process a pending fault event (call from task context).
     */
    virtual void processFaultEvent() = 0;
};

}  // namespace WroRobotSoftware
