/**
 * @file IReflectanceSensor.hpp
 * @brief Hardware-agnostic interface for IR reflectance-based classification.
 *
 * Defines a generic sensor API for reading analog reflectance values and
 * classifying targets based on calibrated voltage ranges. This is reflectance
 * measurement, not true color detection.
 */

#pragma once

#include <cstdint>

#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @enum ReflectanceClass
 * @brief Classification result from reflectance measurement.
 */
enum class ReflectanceClass : uint8_t {
    Unknown = 0,  ///< No match or invalid signal
    Orange = 1,   ///< Orange target detected
    Green = 2     ///< Green target detected
};

/**
 * @class IReflectanceSensor
 * @brief Abstract interface for IR reflectance sensors with classification.
 */
class IReflectanceSensor {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IReflectanceSensor() = default;

    /**
     * @brief Initialize ADC hardware and filter state.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Start periodic sampling task.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t start() = 0;

    /**
     * @brief Stop periodic sampling task.
     * @return esp_err_t ESP_OK on success.
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Get the last raw ADC reading (0-4095).
     * @return Raw ADC count.
     */
    virtual int32_t getRawValue() const noexcept = 0;

    /**
     * @brief Get the filtered voltage value.
     * @return Filtered voltage in volts (0.0-3.3).
     */
    virtual float getFilteredValue() const noexcept = 0;

    /**
     * @brief Get the current classification result.
     * @return Detected reflectance class.
     */
    virtual ReflectanceClass getDetectedClass() const noexcept = 0;

    /**
     * @brief Check if the classification has been stable for the minimum time.
     * @return true if stable.
     */
    virtual bool isStable() const noexcept = 0;
};

}  // namespace WroRobotSoftware
