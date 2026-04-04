/**
 * @file FaultHandler.hpp
 * @brief Fault detection and callback management for DRV8876.
 *
 * Handles the nFAULT GPIO input with ISR-driven detection, atomic flag
 * management, and user callback invocation.
 */

#pragma once

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include <atomic>
#include <functional>

namespace WroRobotSoftware {
namespace DRV8876 {

/**
 * @class FaultHandler
 * @brief Manages DRV8876 fault detection via nFAULT GPIO and ISR.
 */
class FaultHandler {
  public:
    FaultHandler() = default;
    ~FaultHandler() = default;

    FaultHandler(const FaultHandler&) = delete;
    FaultHandler& operator=(const FaultHandler&) = delete;
    FaultHandler(FaultHandler&& other) noexcept;
    FaultHandler& operator=(FaultHandler&& other) noexcept;

    /**
     * @brief Initialize nFAULT GPIO as input with negative-edge interrupt.
     * @param nFaultPin GPIO for nFAULT signal.
     * @return ESP_OK on success.
     */
    esp_err_t init(gpio_num_t nFaultPin);

    /**
     * @brief Check if a fault has been triggered.
     * @return true if fault is active.
     */
    bool isFaultTriggered() const;

    /**
     * @brief Clear the internal fault flag.
     */
    void clearFaultFlag();

    /**
     * @brief Atomically read and clear the fault flag.
     * @return true if a fault was present before clearing.
     */
    bool getAndClearFault();

    /**
     * @brief Register a user callback invoked on fault events.
     * @param cb Callback function or nullptr to disable.
     */
    void setFaultCallback(const std::function<void()>& cb);

    /**
     * @brief Process a pending fault: invoke callback if fault was triggered.
     */
    void processFaultEvent();

  private:
    /**
     * @brief ISR handler for nFAULT negative edge.
     * @param arg Pointer to FaultHandler instance.
     */
    static void IRAM_ATTR faultISR(void* arg);

    gpio_num_t nFaultPin = GPIO_NUM_NC;      ///< Fault detect pin
    std::atomic_bool faultTriggered{false};  ///< Atomic fault flag
    std::function<void()> faultCallback;     ///< User fault callback
    bool initialized = false;                ///< Initialization flag

    static constexpr const char* TAG = "FaultHandler";
};

}  // namespace DRV8876
}  // namespace WroRobotSoftware
