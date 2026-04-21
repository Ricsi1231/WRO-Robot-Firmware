/**
 * @file ServoChannel.hpp
 * @brief Single servo motor control via LEDC PWM channel.
 *
 * Manages one LEDC channel for a single servo, handling angle-to-pulse-width
 * conversion, calibration, inversion, and safe clamping.
 */

#pragma once

#include <cstdint>

#include "driver/ledc.h"
#include "esp_err.h"

namespace WroRobotSoftware {
namespace Servo {

/**
 * @struct ServoChannelConfig
 * @brief Per-servo configuration parameters.
 */
struct ServoChannelConfig {
    gpio_num_t gpioPin;                ///< PWM output GPIO
    ledc_channel_t ledcChannel;        ///< LEDC channel (0-7)
    float minPulseUs = 1000.0f;        ///< Pulse width at minimum angle (µs)
    float maxPulseUs = 2000.0f;        ///< Pulse width at maximum angle (µs)
    float minAngleDeg = 0.0f;          ///< Minimum allowed angle (degrees)
    float maxAngleDeg = 180.0f;        ///< Maximum allowed angle (degrees)
    float initialAngleDeg = 90.0f;     ///< Safe default position on init
    bool inverted = false;             ///< Reverse angle mapping for flipped mounting
    float maxSpeedDegPerSec = 300.0f;  ///< Slew rate limit (deg/sec, 0 = unlimited)
};

/**
 * @class ServoChannel
 * @brief Controls a single servo motor via one LEDC PWM channel.
 *
 * Does not own the LEDC timer — the ServoManager configures the shared timer
 * and passes timer parameters during init.
 */
class ServoChannel {
  public:
    ServoChannel();
    ~ServoChannel();

    ServoChannel(const ServoChannel&) = delete;
    ServoChannel& operator=(const ServoChannel&) = delete;
    ServoChannel(ServoChannel&& other) noexcept;
    ServoChannel& operator=(ServoChannel&& other) noexcept;

    /**
     * @brief Initialize the LEDC channel for this servo.
     * @param config Per-servo configuration.
     * @param timer LEDC timer to bind to (shared, configured by manager).
     * @param resolution Timer resolution (for tick calculations).
     * @param frequencyHz PWM frequency in Hz (typically 50).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t init(const ServoChannelConfig& config, ledc_timer_t timer, ledc_timer_bit_t resolution, uint32_t frequencyHz);

    /**
     * @brief Set the servo angle immediately and write PWM.
     * @param angleDeg Target angle in degrees (clamped to valid range).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t setAngle(float angleDeg);

    /**
     * @brief Get the current servo angle.
     * @return Current angle in degrees.
     */
    float getAngle() const;

    /**
     * @brief Set the ramp target angle without writing PWM.
     * @param angleDeg Target angle in degrees (clamped to valid range).
     */
    void setTargetAngle(float angleDeg);

    /**
     * @brief Get the target angle for ramping.
     * @return Target angle in degrees.
     */
    float getTargetAngle() const;

    /**
     * @brief Step the current angle toward the target by at most maxDeltaDeg.
     * @param maxDeltaDeg Maximum angle change for this step.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t stepTowardTarget(float maxDeltaDeg);

    /**
     * @brief Check if the current angle has reached the target.
     * @return true if at target.
     */
    bool isAtTarget() const;

    /**
     * @brief Stop PWM output (detach servo to save power).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t detach();

    /**
     * @brief Resume PWM output at current angle.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t attach();

    /**
     * @brief Check if the servo PWM output is active.
     * @return true if attached.
     */
    bool isAttached() const;

    /**
     * @brief Get the servo configuration.
     * @return Current configuration.
     */
    ServoChannelConfig getConfig() const;

  private:
    /**
     * @brief Clamp angle to the configured valid range.
     * @param angleDeg Input angle.
     * @return Clamped angle.
     */
    float clampAngle(float angleDeg) const;

    /**
     * @brief Convert angle to LEDC duty ticks.
     * @param angleDeg Servo angle in degrees.
     * @return Duty cycle in ticks.
     */
    uint32_t angleToPulseTicks(float angleDeg) const;

    /**
     * @brief Write a duty tick value to the LEDC channel.
     * @param ticks Duty cycle in ticks.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t writePwm(uint32_t ticks);

    /// Tolerance in degrees to consider angle as "at target"
    static constexpr float ANGLE_TOLERANCE_DEG = 0.1f;

    static constexpr const char* TAG = "ServoChannel";

    ServoChannelConfig config;      ///< Servo configuration
    float currentAngleDeg = 90.0f;  ///< Last commanded angle
    float targetAngleDeg = 90.0f;   ///< Ramp target angle
    bool attached = false;          ///< PWM output active
    bool initialized = false;       ///< Init completed

    uint32_t totalTicks = 0;    ///< Max duty ticks: (1 << resolution) - 1
    float periodUs = 20000.0f;  ///< PWM period in µs: 1e6 / frequencyHz
};

}  // namespace Servo
}  // namespace WroRobotSoftware
