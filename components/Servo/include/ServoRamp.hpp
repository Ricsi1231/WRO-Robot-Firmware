/**
 * @file ServoRamp.hpp
 * @brief Non-blocking angle interpolation for servo motion control.
 *
 * Provides smooth angle transitions via tick-based stepping. Unlike the
 * DRV8876 MotionRamp which blocks with vTaskDelay, ServoRamp is designed
 * to be called periodically by a FreeRTOS task.
 */

#pragma once

#include <cstdint>

namespace WroRobotSoftware {
namespace Servo {

/**
 * @class ServoRamp
 * @brief Non-blocking angle interpolation for smooth servo movement.
 *
 * The manager task calls step() each update period to advance the angle
 * incrementally toward the target.
 */
class ServoRamp {
  public:
    ServoRamp();
    ~ServoRamp();

    ServoRamp(const ServoRamp&) = default;
    ServoRamp& operator=(const ServoRamp&) = default;
    ServoRamp(ServoRamp&&) = default;
    ServoRamp& operator=(ServoRamp&&) = default;

    /**
     * @brief Start a slew-rate-limited ramp to the target angle.
     * @param fromAngle Current angle in degrees.
     * @param toAngle Target angle in degrees.
     * @param maxSpeedDegPerSec Maximum angular speed (deg/sec).
     */
    void startRamp(float fromAngle, float toAngle, float maxSpeedDegPerSec);

    /**
     * @brief Start a timed ramp to the target angle over a fixed duration.
     * @param fromAngle Current angle in degrees.
     * @param toAngle Target angle in degrees.
     * @param durationMs Duration of the transition in milliseconds.
     */
    void startTimedRamp(float fromAngle, float toAngle, uint32_t durationMs);

    /**
     * @brief Compute the next angle after one time step.
     * @param currentAngle Current servo angle in degrees.
     * @param deltaTimeSec Time elapsed since last step in seconds.
     * @return Next angle in degrees (clamped to target).
     */
    float step(float currentAngle, float deltaTimeSec) const;

    /**
     * @brief Check if the ramp has reached its target.
     * @param currentAngle Current servo angle in degrees.
     * @return true if the angle is within tolerance of the target.
     */
    bool isComplete(float currentAngle) const;

    /**
     * @brief Cancel the active ramp.
     */
    void cancel();

    /**
     * @brief Get the ramp target angle.
     * @return Target angle in degrees.
     */
    float getTarget() const;

    /**
     * @brief Check if a ramp is currently active.
     * @return true if ramping.
     */
    bool isActive() const;

  private:
    /// Tolerance in degrees to consider the angle as "at target"
    static constexpr float ANGLE_TOLERANCE_DEG = 0.1f;

    /// Minimum duration to prevent division by zero (ms)
    static constexpr uint32_t MIN_DURATION_MS = 1;

    static constexpr const char* TAG = "ServoRamp";

    float targetAngleDeg = 0.0f;  ///< Ramp destination
    float speedDegPerSec = 0.0f;  ///< Angular speed for this ramp
    float direction = 0.0f;       ///< +1.0 or -1.0
    bool active = false;          ///< Whether a ramp is in progress
};

}  // namespace Servo
}  // namespace WroRobotSoftware
