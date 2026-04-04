/**
 * @file PwmController.hpp
 * @brief LEDC PWM timer and duty cycle management for DRV8876.
 *
 * Encapsulates LEDC timer configuration, channel duty cycle writes,
 * frequency management, and mutex-protected peripheral access.
 */

#pragma once

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <inttypes.h>

namespace WroRobotSoftware {
namespace DRV8876 {

/**
 * @class PwmController
 * @brief Manages LEDC timer, channel, and duty cycle for DRV8876 EN pin.
 */
class PwmController {
  public:
    PwmController();
    ~PwmController();

    PwmController(const PwmController&) = delete;
    PwmController& operator=(const PwmController&) = delete;
    PwmController(PwmController&& other) noexcept;
    PwmController& operator=(PwmController&& other) noexcept;

    /**
     * @brief Initialize LEDC timer and channel.
     * @param enPin GPIO for EN (PWM output).
     * @param channel LEDC channel to use.
     * @param resolution PWM resolution (e.g., LEDC_TIMER_10_BIT).
     * @param frequency PWM frequency in Hz.
     * @return ESP_OK on success.
     */
    esp_err_t init(gpio_num_t enPin, ledc_channel_t channel, ledc_timer_bit_t resolution, uint32_t frequency);

    /**
     * @brief Set duty cycle from a 0–255 value, mutex-protected.
     * @param duty Duty value (0–255).
     * @return ESP_OK on success.
     */
    esp_err_t setDuty(uint8_t duty);

    /**
     * @brief Set duty cycle directly in timer ticks.
     * @param dutyTicks Number of ticks for duty cycle.
     * @return ESP_OK on success.
     */
    esp_err_t setDutyTicks(uint32_t dutyTicks);

    /**
     * @brief Convert a 0–100% value to timer ticks.
     * @param percent Speed percentage (0–100).
     * @return Corresponding tick count.
     */
    uint32_t percentToTicks(uint8_t percent) const;

    /**
     * @brief Reconfigure LEDC timer resolution and frequency.
     * @param resolution New PWM resolution.
     * @param frequency New frequency in Hz.
     * @return ESP_OK on success.
     */
    esp_err_t setTimerConfig(ledc_timer_bit_t resolution, uint32_t frequency);

    /**
     * @brief Change PWM frequency while keeping current resolution.
     * @param frequency New frequency in Hz.
     * @return ESP_OK on success.
     */
    esp_err_t setFrequency(uint32_t frequency);

    /**
     * @brief Configure allowed frequency range.
     * @param minHz Minimum allowed frequency in Hz.
     * @param maxHz Maximum allowed frequency in Hz.
     */
    void setFrequencyRange(uint32_t minHz, uint32_t maxHz);

    /**
     * @brief Get configured minimum PWM frequency.
     * @return Minimum frequency in Hz.
     */
    uint32_t getMinFrequency() const;

    /**
     * @brief Get configured maximum PWM frequency.
     * @return Maximum frequency in Hz.
     */
    uint32_t getMaxFrequency() const;

    /**
     * @brief Get current PWM resolution.
     * @return LEDC timer bit resolution.
     */
    ledc_timer_bit_t getResolution() const;

    /**
     * @brief Get current PWM frequency.
     * @return Frequency in Hz.
     */
    uint32_t getFrequency() const;

  private:
    /**
     * @brief Acquire LEDC mutex with timeout.
     * @param timeoutMs Timeout in milliseconds.
     * @return true if lock acquired.
     */
    bool lockLedc(uint32_t timeoutMs);

    /**
     * @brief Release LEDC mutex.
     */
    void unlockLedc();

    ledc_channel_t pwmChannel = LEDC_CHANNEL_0;       ///< LEDC channel
    ledc_timer_bit_t resolution = LEDC_TIMER_10_BIT;  ///< PWM resolution
    uint32_t frequency = 20000;                       ///< Current PWM frequency (Hz)
    uint32_t minFrequency = 100;                      ///< Minimum allowed frequency (Hz)
    uint32_t maxFrequency = 100000;                   ///< Maximum allowed frequency (Hz)
    uint32_t maxDuty = 0;                             ///< Max duty in ticks

    SemaphoreHandle_t ledcMutex = nullptr;  ///< Mutex for LEDC access
    uint32_t ledcMutexTimeoutMs = 50;       ///< Mutex timeout (ms)
    bool initialized = false;               ///< Initialization flag

    static constexpr const char* TAG = "PwmController";
};

}  // namespace DRV8876
}  // namespace WroRobotSoftware
