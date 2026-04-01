/**
 * @file IRGBLed.hpp
 * @brief Hardware-agnostic interface for RGB LED controllers.
 *
 * Defines a generic RGB LED API that works with any LED driver
 * (WS2812, SK6812, PWM-driven RGB, etc.). Concrete implementations
 * inherit from this interface and provide device-specific functionality.
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @enum PresetColor
 * @brief Predefined color presets for convenience.
 */
enum class PresetColor {
    RED,      ///< Pure red
    GREEN,    ///< Pure green
    BLUE,     ///< Pure blue
    YELLOW,   ///< Red + green
    CYAN,     ///< Green + blue
    MAGENTA,  ///< Red + blue
    WHITE     ///< All channels on
};

/**
 * @class IRGBLed
 * @brief Abstract interface for controlling an RGB LED.
 */
class IRGBLed {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IRGBLed() = default;

    /**
     * @brief Initialize the LED hardware.
     * @return esp_err_t ESP_OK on success, else error code.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Smoothly transition to a target color over a given duration.
     * @param targetRed Target red channel value (0-255).
     * @param targetGreen Target green channel value (0-255).
     * @param targetBlue Target blue channel value (0-255).
     * @param durationMs Fade duration in milliseconds.
     */
    virtual void fadeToColor(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue, uint16_t durationMs) = 0;

    /**
     * @brief Set the LED to a predefined color preset.
     * @param color Preset color to apply.
     */
    virtual void setColor(PresetColor color) = 0;

    /**
     * @brief Blink the LED a specified number of times.
     * @param blinkDelay Delay between blinks in milliseconds.
     * @param blinkTimes Number of blink cycles.
     */
    virtual void blinkLed(uint8_t blinkDelay, uint8_t blinkTimes) = 0;

    /**
     * @brief Set the overall LED brightness.
     * @param percent Brightness level (0-100%).
     */
    virtual void setBrightness(uint8_t percent) = 0;

    /**
     * @brief Turn the LED off (all channels to zero).
     */
    virtual void turnOffLed() = 0;

    /**
     * @brief Turn the LED on (restore previous color).
     */
    virtual void turnOnLed() = 0;

    /**
     * @brief Check if the LED is currently on.
     * @return true if on, false if off.
     */
    virtual bool isOn() const = 0;
};

}  // namespace WroRobotSoftware
