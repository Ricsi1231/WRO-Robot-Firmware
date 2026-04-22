/**
 * @file GpioInterruptManager.hpp
 * @brief Centralized GPIO interrupt manager with ISR-safe event dispatch.
 *
 * Provides a single point of GPIO interrupt registration with automatic
 * ISR-to-task dispatch via FreeRTOS queue. ISR does minimal work (enqueue
 * event), callbacks execute safely in task context.
 *
 * Only one instance should be created per application. Owns the global
 * GPIO ISR service installation.
 */

#pragma once

#include <array>
#include <cstdint>
#include <functional>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#include "IGpioInterruptManager.hpp"

namespace WroRobotSoftware {
namespace GpioInterruptManager {

/// Maximum number of GPIO pins on this SoC
static constexpr size_t MAX_GPIO_PINS = SOC_GPIO_PIN_COUNT;

/**
 * @struct GpioInterruptManagerConfig
 * @brief Configuration for the GPIO interrupt manager.
 */
struct GpioInterruptManagerConfig {
  uint32_t eventQueueDepth = 32;     ///< FreeRTOS queue depth for GPIO events
  uint32_t taskStackSize = 4096;     ///< Dispatch task stack size (bytes)
  UBaseType_t taskPriority = 5;      ///< Dispatch task FreeRTOS priority
  int isrAllocFlags = 0;             ///< Flags for gpio_install_isr_service
};

/**
 * @struct GpioEvent
 * @brief An interrupt event pushed from ISR to the dispatch queue.
 */
struct GpioEvent {
  gpio_num_t pin;         ///< GPIO pin that triggered
  int64_t timestampUs;    ///< Timestamp from esp_timer_get_time() at ISR entry
};

/**
 * @class GpioInterruptManager
 * @brief Manages GPIO interrupts with centralized ISR dispatch.
 *
 * Registers per-pin ISR handlers that enqueue events to a FreeRTOS queue.
 * A dispatch task reads the queue and invokes registered callbacks in task
 * context, ensuring safety for logging, I2C, SPI, and other blocking ops.
 */
class GpioInterruptManager : public IGpioInterruptManager {
 public:
  /**
   * @brief Construct the interrupt manager.
   * @param config Manager configuration.
   */
  explicit GpioInterruptManager(const GpioInterruptManagerConfig& config);

  /**
   * @brief Destructor. Stops task and unregisters all pins.
   */
  ~GpioInterruptManager() override;

  GpioInterruptManager(const GpioInterruptManager&) = delete;
  GpioInterruptManager& operator=(const GpioInterruptManager&) = delete;

  /**
   * @brief Install ISR service, create queue and dispatch task.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t init() override;

  /**
   * @brief Register a GPIO interrupt with callback.
   * @param pin GPIO pin number.
   * @param edge Trigger edge type.
   * @param pull Pull resistor configuration.
   * @param callback Function called in task context on interrupt.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t registerInterrupt(gpio_num_t pin, EdgeType edge, GpioPull pull,
                              std::function<void(gpio_num_t)> callback) override;

  /**
   * @brief Unregister a GPIO interrupt.
   * @param pin GPIO pin number.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t unregisterInterrupt(gpio_num_t pin) override;

  /**
   * @brief Enable a previously disabled interrupt.
   * @param pin GPIO pin number.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t enableInterrupt(gpio_num_t pin) override;

  /**
   * @brief Temporarily disable an interrupt without unregistering.
   * @param pin GPIO pin number.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t disableInterrupt(gpio_num_t pin) override;

 private:
  /**
   * @struct PinRegistration
   * @brief Registration state for a single GPIO pin.
   */
  struct PinRegistration {
    std::function<void(gpio_num_t)> callback;  ///< User callback
    EdgeType edge;                              ///< Configured edge type
    bool registered = false;                    ///< Slot in use
    bool enabled = false;                       ///< Interrupt currently active
  };

  /**
   * @struct IsrContext
   * @brief Minimal ISR-safe context passed to the shared ISR handler.
   */
  struct IsrContext {
    QueueHandle_t queue = nullptr;  ///< Pointer to event queue
    gpio_num_t pin = GPIO_NUM_NC;   ///< Pin number for this slot
  };

  /**
   * @brief Shared ISR handler for all registered pins.
   * @param arg Pointer to IsrContext for the triggering pin.
   */
  static void IRAM_ATTR gpioISR(void* arg);

  /**
   * @brief FreeRTOS task that dispatches events to callbacks.
   * @param arg Pointer to GpioInterruptManager instance.
   */
  static void dispatchTask(void* arg);

  /**
   * @brief Convert EdgeType to ESP-IDF gpio_int_type_t.
   * @param edge Edge type.
   * @return Corresponding ESP-IDF interrupt type.
   */
  static gpio_int_type_t toGpioIntrType(EdgeType edge);

  /**
   * @brief Check if a pin number is within valid range.
   * @param pin GPIO pin to validate.
   * @return true if valid.
   */
  bool isValidPin(gpio_num_t pin) const;

  static constexpr const char* TAG = "GpioInterruptManager";

  GpioInterruptManagerConfig config;                            ///< Manager config
  std::array<PinRegistration, MAX_GPIO_PINS> registrations{};   ///< Per-pin registrations
  std::array<IsrContext, MAX_GPIO_PINS> isrContexts{};          ///< Per-pin ISR contexts
  QueueHandle_t eventQueue = nullptr;                           ///< Event queue
  TaskHandle_t taskHandle = nullptr;                            ///< Dispatch task handle
  bool initialized = false;                                     ///< Init completed
};

}  // namespace GpioInterruptManager
}  // namespace WroRobotSoftware
