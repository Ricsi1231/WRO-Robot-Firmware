/**
 * @file IGpioInterruptManager.hpp
 * @brief Hardware-agnostic interface for centralized GPIO interrupt management.
 *
 * Provides a unified API for registering, enabling, and disabling GPIO
 * interrupts with callback dispatch in task context.
 */

#pragma once

#include <cstdint>
#include <functional>

#include "driver/gpio.h"
#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @enum EdgeType
 * @brief GPIO interrupt trigger edge type.
 */
enum class EdgeType : uint8_t {
    Rising,   ///< Trigger on rising edge (low → high)
    Falling,  ///< Trigger on falling edge (high → low)
    Both      ///< Trigger on both edges
};

/**
 * @enum GpioPull
 * @brief GPIO pull resistor configuration.
 */
enum class GpioPull : uint8_t {
    None,  ///< No pull resistor
    Up,    ///< Internal pull-up enabled
    Down   ///< Internal pull-down enabled
};

/**
 * @class IGpioInterruptManager
 * @brief Abstract interface for GPIO interrupt management.
 */
class IGpioInterruptManager {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IGpioInterruptManager() = default;

    /**
     * @brief Initialize the interrupt manager and ISR service.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Register a GPIO interrupt with callback.
     * @param pin GPIO pin number.
     * @param edge Trigger edge type.
     * @param pull Pull resistor configuration.
     * @param callback Function called in task context when interrupt fires.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin invalid,
     *         ESP_ERR_INVALID_STATE if pin already registered.
     */
    virtual esp_err_t registerInterrupt(gpio_num_t pin, EdgeType edge, GpioPull pull, std::function<void(gpio_num_t)> callback) = 0;

    /**
     * @brief Unregister a GPIO interrupt and release the pin.
     * @param pin GPIO pin number.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pin not
     *         registered.
     */
    virtual esp_err_t unregisterInterrupt(gpio_num_t pin) = 0;

    /**
     * @brief Enable a previously disabled interrupt.
     * @param pin GPIO pin number.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t enableInterrupt(gpio_num_t pin) = 0;

    /**
     * @brief Temporarily disable an interrupt without unregistering.
     * @param pin GPIO pin number.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t disableInterrupt(gpio_num_t pin) = 0;
};

}  // namespace WroRobotSoftware
