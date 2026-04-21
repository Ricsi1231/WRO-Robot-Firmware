/**
 * @file IServoDriver.hpp
 * @brief Hardware-agnostic interface for servo motor control.
 *
 * Defines a generic servo driver API for position-based control of standard
 * hobby servos. Supports multiple servos via a servo ID, smooth transitions,
 * and attach/detach for power management.
 */

#pragma once

#include <cstdint>

#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @class IServoDriver
 * @brief Abstract interface for controlling servo motors.
 */
class IServoDriver {
 public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~IServoDriver() = default;

  /**
   * @brief Initialize the servo driver hardware.
   * @return esp_err_t ESP_OK on success, else error code.
   */
  virtual esp_err_t init() = 0;

  /**
   * @brief Set a servo to the specified angle using slew rate limiting.
   * @param servoId Servo channel identifier.
   * @param angleDeg Target angle in degrees.
   * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if servoId
   *         is invalid.
   */
  virtual esp_err_t setAngle(uint8_t servoId, float angleDeg) = 0;

  /**
   * @brief Smoothly move a servo to the specified angle over a given duration.
   * @param servoId Servo channel identifier.
   * @param angleDeg Target angle in degrees.
   * @param durationMs Duration of the transition in milliseconds.
   * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if servoId
   *         is invalid.
   */
  virtual esp_err_t setAngleSmooth(uint8_t servoId, float angleDeg,
                                   uint32_t durationMs) = 0;

  /**
   * @brief Get the current angle of a servo.
   * @param servoId Servo channel identifier.
   * @return Current angle in degrees.
   */
  virtual float getAngle(uint8_t servoId) const = 0;

  /**
   * @brief Get the target angle a servo is moving toward.
   * @param servoId Servo channel identifier.
   * @return Target angle in degrees.
   */
  virtual float getTargetAngle(uint8_t servoId) const = 0;

  /**
   * @brief Check if a servo is currently in motion.
   * @param servoId Servo channel identifier.
   * @return true if the servo has not yet reached its target angle.
   */
  virtual bool isMoving(uint8_t servoId) const = 0;

  /**
   * @brief Detach a servo (stop PWM output to save power).
   * @param servoId Servo channel identifier.
   * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if servoId
   *         is invalid.
   */
  virtual esp_err_t detach(uint8_t servoId) = 0;

  /**
   * @brief Re-attach a previously detached servo (resume PWM output).
   * @param servoId Servo channel identifier.
   * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if servoId
   *         is invalid.
   */
  virtual esp_err_t attach(uint8_t servoId) = 0;
};

}  // namespace WroRobotSoftware
