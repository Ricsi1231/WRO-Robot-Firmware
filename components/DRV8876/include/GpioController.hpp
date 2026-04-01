/**
 * @file GpioController.hpp
 * @brief GPIO management for DRV8876 direction and sleep pins.
 *
 * Configures and controls the PH (direction) and nSLEEP (enable) GPIO outputs,
 * as well as direct GPIO writes needed for coast/brake operations.
 */

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

namespace WroRobotSoftware {
namespace DRV8876 {

/**
 * @class GpioController
 * @brief Manages direction and sleep GPIO pins for DRV8876.
 */
class GpioController {
  public:
    GpioController() = default;
    ~GpioController() = default;

    GpioController(const GpioController&) = delete;
    GpioController& operator=(const GpioController&) = delete;
    GpioController(GpioController&& other) noexcept;
    GpioController& operator=(GpioController&& other) noexcept;

    /**
     * @brief Initialize PH and nSLEEP GPIO pins as outputs.
     * @param phPin Direction control GPIO.
     * @param nSleepPin Driver enable GPIO.
     * @return ESP_OK on success.
     */
    esp_err_t init(gpio_num_t phPin, gpio_num_t nSleepPin);

    /**
     * @brief Set the PH pin level for motor direction.
     * @param level true = RIGHT, false = LEFT.
     */
    void setDirection(bool level);

    /**
     * @brief Set the nSLEEP pin to wake or sleep the driver.
     * @param awake true = wake (high), false = sleep (low).
     */
    void setSleep(bool awake);

    /**
     * @brief Set an arbitrary GPIO level (used for coast/brake on EN pin).
     * @param pin GPIO pin to write.
     * @param level Desired logic level.
     */
    void setLevel(gpio_num_t pin, bool level);

  private:
    gpio_num_t phPin = GPIO_NUM_NC;      ///< Direction control pin
    gpio_num_t nSleepPin = GPIO_NUM_NC;  ///< Driver enable pin
    bool initialized = false;            ///< Initialization flag

    static constexpr const char* TAG = "GpioController";
};

}  // namespace DRV8876
}  // namespace WroRobotSoftware
