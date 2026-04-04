/**
 * @file MotionRamp.hpp
 * @brief Speed ramping logic for DRV8876 motor driver.
 *
 * Provides graduated speed transitions using configurable step size and delay.
 * Decoupled from hardware — operates via a callback to apply each speed step.
 */

#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <functional>

namespace WroRobotSoftware {
namespace DRV8876 {

/**
 * @class MotionRamp
 * @brief Graduated speed change logic with configurable step size and delay.
 */
class MotionRamp {
  public:
    /**
     * @brief Construct a MotionRamp with ramp parameters.
     * @param rampStepPercent Step size in percent per ramp iteration.
     * @param rampStepDelayMs Delay in ms between ramp steps.
     * @param minEffectivePwmPercent Minimum non-zero effective PWM percentage.
     */
    MotionRamp(uint8_t rampStepPercent, uint32_t rampStepDelayMs, uint8_t minEffectivePwmPercent);

    MotionRamp() = default;
    ~MotionRamp() = default;

    MotionRamp(const MotionRamp&) = default;
    MotionRamp& operator=(const MotionRamp&) = default;
    MotionRamp(MotionRamp&&) = default;
    MotionRamp& operator=(MotionRamp&&) = default;

    /**
     * @brief Ramp from current speed to target using configured step size.
     * @param targetPercent Target speed (0–100%).
     * @param currentSpeed Current motor speed (0–100%).
     * @param setSpeedFn Callback to apply each speed step.
     * @return ESP_OK on success.
     */
    esp_err_t rampTo(uint8_t targetPercent, uint8_t currentSpeed, const std::function<esp_err_t(uint8_t)>& setSpeedFn);

    /**
     * @brief Ramp from current speed to target over a fixed time duration.
     * @param targetPercent Target speed (0–100%).
     * @param rampTimeMs Total ramp duration in ms.
     * @param currentSpeed Current motor speed (0–100%).
     * @param setSpeedFn Callback to apply each speed step.
     * @return ESP_OK on success.
     */
    esp_err_t timedRamp(uint8_t targetPercent, uint32_t rampTimeMs, uint8_t currentSpeed, const std::function<esp_err_t(uint8_t)>& setSpeedFn);

  private:
    uint8_t rampStepPercent = 5;         ///< Step size per ramp iteration (%)
    uint32_t rampStepDelayMs = 5;        ///< Delay between steps (ms)
    uint8_t minEffectivePwmPercent = 3;  ///< Minimum effective PWM (%)

    static constexpr const char* TAG = "MotionRamp";
};

}  // namespace DRV8876
}  // namespace WroRobotSoftware
