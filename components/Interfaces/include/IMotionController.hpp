/**
 * @file IMotionController.hpp
 * @brief Hardware-agnostic interface for high-level robot motion control.
 *
 * Defines a generic motion controller API for car-like robots with a single
 * drive motor and steering mechanism. Abstracts velocity and steering angle
 * into a unified control interface.
 */

#pragma once

#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @class IMotionController
 * @brief Abstract interface for robot motion control.
 */
class IMotionController {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IMotionController() = default;

    /**
     * @brief Initialize the motion controller and all owned actuators.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Set the drive velocity.
     * @param velocity Normalized velocity [-1.0, +1.0]. Positive = forward,
     *        negative = backward.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t setVelocity(float velocity) = 0;

    /**
     * @brief Set the steering angle.
     * @param angleDeg Steering angle in degrees. Negative = left, positive =
     *        right. Clamped to configured limits.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t setSteeringAngle(float angleDeg) = 0;

    /**
     * @brief Set both velocity and steering angle simultaneously.
     * @param velocity Normalized velocity [-1.0, +1.0].
     * @param angleDeg Steering angle in degrees.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t setMotion(float velocity, float angleDeg) = 0;

    /**
     * @brief Stop the drive motor. Steering behavior depends on configuration.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Immediately brake the drive motor without touching steering.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t emergencyStop() = 0;

    /**
     * @brief Get the current velocity.
     * @return Normalized velocity [-1.0, +1.0].
     */
    virtual float getVelocity() const = 0;

    /**
     * @brief Get the current steering angle.
     * @return Steering angle in degrees.
     */
    virtual float getSteeringAngle() const = 0;

    /**
     * @brief Check if the robot is currently in motion.
     * @return true if the drive motor is running.
     */
    virtual bool isMoving() const = 0;
};

}  // namespace WroRobotSoftware
