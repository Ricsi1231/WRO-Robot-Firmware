/**
 * @file MotionController.hpp
 * @brief High-level motion controller for a car-like robot.
 *
 * Composes a DC motor driver (DRV8876) and a steering servo (ServoManager)
 * into a unified velocity + steering angle interface. Owns both actuators
 * via std::unique_ptr for correct resource management.
 */

#pragma once

#include <cstdint>
#include <memory>

#include "esp_err.h"

#include "DRV8876.hpp"
#include "IMotionController.hpp"
#include "IMotorDriver.hpp"
#include "ServoManager.hpp"

namespace WroRobotSoftware {
namespace MotionController {

/**
 * @struct MotionControllerConfig
 * @brief Configuration parameters for the motion controller.
 */
struct MotionControllerConfig {
    float maxSteeringAngleDeg = 30.0f;                        ///< Maximum steering angle from center (symmetric, degrees)
    float servoCenterAngleDeg = 90.0f;                        ///< Servo angle that corresponds to straight ahead (degrees)
    uint8_t steeringServoId = 0;                              ///< Servo channel ID within the ServoManager
    MotorDirection forwardDirection = MotorDirection::RIGHT;  ///< MotorDirection that maps to positive velocity
    bool centerSteeringOnStop = true;                         ///< Whether stop() also centers the steering
    uint32_t velocityRampTimeMs = 0;                          ///< Motor ramp time (ms, 0 = instant)
    uint32_t steeringSmoothTimeMs = 0;                        ///< Servo smooth transition time (ms, 0 = instant)
};

/**
 * @class MotionController
 * @brief Orchestrates drive motor and steering servo for car-like motion.
 *
 * Translates normalized velocity [-1, +1] and steering angle to motor
 * speed/direction and servo position. Handles safe direction reversals
 * and steering limits.
 */
class MotionController : public IMotionController {
  public:
    /**
     * @brief Construct the motion controller with owned actuators.
     * @param config Motion controller configuration.
     * @param motorDriver Owned DC motor driver (transferred via unique_ptr).
     * @param servoManager Owned servo manager (transferred via unique_ptr).
     */
    MotionController(const MotionControllerConfig& config, std::unique_ptr<DRV8876::DRV8876> motorDriver, std::unique_ptr<Servo::ServoManager> servoManager);

    /**
     * @brief Destructor.
     */
    ~MotionController() override;

    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;
    MotionController(MotionController&&) = delete;
    MotionController& operator=(MotionController&&) = delete;

    /**
     * @brief Initialize motor driver and servo manager.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t init() override;

    /**
     * @brief Set drive velocity with automatic direction handling.
     * @param velocity Normalized velocity [-1.0, +1.0].
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t setVelocity(float velocity) override;

    /**
     * @brief Set steering angle relative to center.
     * @param angleDeg Steering angle in degrees (negative = left, positive = right).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t setSteeringAngle(float angleDeg) override;

    /**
     * @brief Set velocity and steering angle simultaneously.
     * @param velocity Normalized velocity [-1.0, +1.0].
     * @param angleDeg Steering angle in degrees.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t setMotion(float velocity, float angleDeg) override;

    /**
     * @brief Stop the drive motor. Optionally centers steering.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t stop() override;

    /**
     * @brief Immediately brake the motor without touching steering.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t emergencyStop() override;

    /**
     * @brief Get the current velocity.
     * @return Normalized velocity [-1.0, +1.0].
     */
    float getVelocity() const override;

    /**
     * @brief Get the current steering angle.
     * @return Steering angle in degrees.
     */
    float getSteeringAngle() const override;

    /**
     * @brief Check if the drive motor is running.
     * @return true if motor is active.
     */
    bool isMoving() const override;

  private:
    /**
     * @brief Apply velocity to the motor driver.
     * @param velocity Clamped velocity value.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t applyVelocity(float velocity);

    /**
     * @brief Apply steering angle to the servo.
     * @param angleDeg Clamped steering angle.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t applySteering(float angleDeg);

    /**
     * @brief Clamp velocity to valid range.
     * @param velocity Input velocity.
     * @return Clamped velocity.
     */
    float clampVelocity(float velocity) const;

    /**
     * @brief Clamp steering angle to configured limits.
     * @param angleDeg Input angle.
     * @return Clamped angle.
     */
    float clampSteeringAngle(float angleDeg) const;

    /**
     * @brief Convert steering angle to servo angle.
     * @param steeringAngleDeg Steering angle relative to center.
     * @return Absolute servo angle in degrees.
     */
    float steeringToServoAngle(float steeringAngleDeg) const;

    /**
     * @brief Determine motor direction from velocity sign.
     * @param velocity Signed velocity value.
     * @return Corresponding MotorDirection.
     */
    MotorDirection velocityToDirection(float velocity) const;

    /**
     * @brief Check if new velocity would cause a direction reversal.
     * @param newVelocity Proposed velocity.
     * @return true if direction would change.
     */
    bool directionWouldReverse(float newVelocity) const;

    /// Minimum velocity magnitude
    static constexpr float MIN_VELOCITY = -1.0f;

    /// Maximum velocity magnitude
    static constexpr float MAX_VELOCITY = 1.0f;

    /// Velocity below this threshold is treated as zero
    static constexpr float VELOCITY_DEADZONE = 0.01f;

    /// Multiplier to convert velocity [0, 1] to motor speed [0, 100]
    static constexpr float VELOCITY_TO_SPEED_SCALE = 100.0f;

    static constexpr const char* TAG = "MotionController";

    MotionControllerConfig config;                      ///< Controller configuration
    std::unique_ptr<DRV8876::DRV8876> motorDriver;      ///< Owned drive motor
    std::unique_ptr<Servo::ServoManager> servoManager;  ///< Owned steering servo

    float currentVelocity = 0.0f;       ///< Last commanded velocity
    float currentSteeringAngle = 0.0f;  ///< Last commanded steering angle
    bool initialized = false;           ///< Init completed flag
};

}  // namespace MotionController
}  // namespace WroRobotSoftware
