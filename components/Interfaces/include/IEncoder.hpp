/**
 * @file IEncoder.hpp
 * @brief Hardware-agnostic interface for rotary encoders.
 *
 * Defines a generic encoder API that works with any rotary encoder type
 * (quadrature, absolute, I2C-based, etc.). Concrete implementations inherit
 * from this interface and provide device-specific functionality.
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @enum EncoderDirection
 * @brief Logical encoder rotation direction.
 */
enum class EncoderDirection : bool {
    LEFT = true,   ///< Counter-clockwise direction
    RIGHT = false  ///< Clockwise direction
};

/**
 * @struct EncoderStats
 * @brief Runtime statistics snapshot for an encoder.
 */
struct EncoderStats {
    int64_t totalCounts = 0;   ///< Accumulated tick count since last reset
    int64_t lastDelta = 0;     ///< Tick delta from the most recent sample period
    float rpmRaw = 0.0f;       ///< Unfiltered instantaneous RPM estimate
    float rpmFiltered = 0.0f;  ///< Filtered RPM estimate
    uint32_t overflows = 0;    ///< Number of counter overflow events
    uint32_t missedEdges = 0;  ///< Number of missed edge events
};

/**
 * @class IEncoder
 * @brief Abstract interface for reading a rotary encoder.
 */
class IEncoder {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IEncoder() = default;

    /**
     * @brief Initialize the encoder hardware.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Start periodic encoder measurements.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t start() = 0;

    /**
     * @brief Stop periodic encoder measurements.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Reset accumulated position to zero.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t resetPosition() = 0;

    /**
     * @brief Set mechanical gear ratio applied to position and RPM readouts.
     * @param ratio Gear ratio (>0). Values != 1.0 scale outputs accordingly.
     */
    virtual void setGearRatio(float ratio) noexcept = 0;

    /**
     * @brief Get current position in raw encoder ticks (signed).
     * @return Accumulated tick count.
     */
    virtual int32_t getPositionTicks() const noexcept = 0;

    /**
     * @brief Get current mechanical position in degrees.
     * @return Position in degrees.
     */
    virtual float getPositionInDegrees() const noexcept = 0;

    /**
     * @brief Get filtered RPM estimate.
     * @return Filtered RPM (signed by direction).
     */
    virtual float getRpm() const noexcept = 0;

    /**
     * @brief Get unfiltered instantaneous RPM.
     * @return Raw RPM estimate (signed).
     */
    virtual float getRpmUnfiltered() const noexcept = 0;

    /**
     * @brief Get rounded integer RPM.
     * @return Rounded RPM (signed).
     */
    virtual int32_t getRpmRounded() const noexcept = 0;

    /**
     * @brief Get stable (debounced) motor direction.
     * @return EncoderDirection enum value.
     */
    virtual EncoderDirection getMotorDirection() const noexcept = 0;

    /**
     * @brief Get immediate/raw direction (minimal debouncing).
     * @return EncoderDirection enum value.
     */
    virtual EncoderDirection getMotorDirectionRaw() const noexcept = 0;

    /**
     * @brief Health flag: true if RPM beneath stall threshold.
     * @return true if considered stalled.
     */
    virtual bool isStalled() const noexcept = 0;

    /**
     * @brief Health flag: true if recent RPM stdev exceeds noise threshold.
     * @return true if considered noisy.
     */
    virtual bool isNoisy() const noexcept = 0;

    /**
     * @brief Health flag: true if near counter saturation.
     * @return true if considered saturated.
     */
    virtual bool isSaturated() const noexcept = 0;

    /**
     * @brief Get a consistent snapshot of runtime statistics.
     * @return EncoderStats struct with counts, RPMs, and counters.
     */
    virtual EncoderStats getStats() const noexcept = 0;
};

}  // namespace WroRobotSoftware
