/**
 * @file DRV8876.hpp
 * @brief DRV8876 Motor Driver Control Class using PWM and GPIO.
 *
 * Provides high-level interface for controlling a DC motor using the DRV8876
 * H-bridge driver via ESP32 GPIO and LEDC PWM peripherals.
 *
 * Internally delegates to sub-components: GpioController, FaultHandler,
 * PwmController, and MotionRamp.
 */

#pragma once

#include <inttypes.h>
#include <functional>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "FaultHandler.hpp"
#include "GpioController.hpp"
#include "IMotorDriver.hpp"
#include "MotionRamp.hpp"
#include "PwmController.hpp"

namespace WroRobotSoftware {
namespace DRV8876 {

using Direction = MotorDirection;

struct DRV8876Config {
    gpio_num_t phPin;           ///< Direction pin
    gpio_num_t enPin;           ///< Enable (PWM) pin
    gpio_num_t nFault;          ///< Fault detect pin
    gpio_num_t nSleep;          ///< Driver enable pin
    ledc_channel_t pwmChannel;  ///< PWM channel used

    ledc_timer_bit_t resolution;     ///< Default PWM resolution
    uint32_t frequency;              ///< Default PWM frequency in Hz
    uint32_t minFrequency;           ///< Minimum allowed PWM frequency in Hz (default 100 Hz)
    uint32_t maxFrequency;           ///< Maximum allowed PWM frequency in Hz (default 100 kHz)
    uint8_t rampStepPercent;         ///< step size in % for safe direction changes
    uint32_t rampStepDelayMs;        ///< delay in ms between ramp steps
    uint8_t minEffectivePwmPercent;  ///< Minimum effective PWM duty (%) to overcome deadband
};

/**
 * @class DRV8876
 * @brief Class to control a DRV8876 H-bridge DC motor driver via ESP32.
 */
class DRV8876 : public IMotorDriver {
  public:
    /**
     * @brief Constructor for DRV8876 class.
     *
     * @param config Configuration structure with pin assignments and PWM parameters.
     */
    explicit DRV8876(const DRV8876Config& config);

    /**
     * @brief Destructor.
     */
    ~DRV8876() override;

    /**
     * @brief Deleted copy constructor to prevent copying of DRV8876 instance.
     */
    DRV8876(const DRV8876&) = delete;

    /**
     * @brief Deleted copy assignment operator to prevent copying of DRV8876 instance.
     */
    DRV8876& operator=(const DRV8876&) = delete;

    /**
     * @brief Move constructor for transferring ownership of a DRV8876 instance.
     * @param other Source instance to move from.
     */
    DRV8876(DRV8876&& other) noexcept;

    /**
     * @brief Move assignment operator for transferring ownership of a DRV8876 instance.
     * @param other Source instance to move from.
     * @return Reference to the assigned DRV8876 instance.
     */
    DRV8876& operator=(DRV8876&& other) noexcept;

    /**
     * @brief Initialize motor driver: configure GPIO, PWM, and fault interrupt.
     *
     * @return esp_err_t ESP_OK if success, else error code.
     */
    esp_err_t init() override;

    /**
     * @brief Get the current DRV8876 configuration.
     * @return Copy of DRV8876Config structure.
     */
    DRV8876Config getConfig() const;

    /**
     * @brief Set the motor rotation direction.
     * @param direction Desired motor direction.
     */
    void setDirection(MotorDirection direction) override;

    /**
     * @brief Reverse the current motor direction.
     */
    void reverseDirection() override;

    /**
     * @brief Safely change direction by ramping speed down, switching, then ramping back up.
     * @param direction Desired new motor direction.
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t setDirectionSafe(MotorDirection direction) override;

    /**
     * @brief Safely reverse direction by ramping speed down, switching, then ramping back up.
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t reverseDirectionSafe() override;

    /**
     * @brief Set motor speed as a percentage (0–100%).
     * @param speed Target speed in percent.
     * @return esp_err_t ESP_OK if success, ESP_ERR_INVALID_ARG if out of range.
     */
    esp_err_t setSpeed(uint8_t speed) override;

    /**
     * @brief Set motor speed with a timed ramp transition.
     * @param targetPercent Target speed in percent.
     * @param rampTimeMs Duration of the ramp in milliseconds.
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t setSpeed(uint8_t targetPercent, uint32_t rampTimeMs) override;

    /**
     * @brief Stop the motor (set duty to zero).
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t stop() override;

    /**
     * @brief Set the motor to coast mode (outputs disabled, free spin).
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t coast() override;

    /**
     * @brief Set the motor to brake mode (outputs shorted).
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t brake() override;

    /**
     * @brief Get the current motor speed.
     * @return Motor speed in percent (0–100).
     */
    uint8_t getMotorSpeed() const override;

    /**
     * @brief Get the current motor direction.
     * @return MotorDirection enum value.
     */
    MotorDirection getMotorDirection() const override;

    /**
     * @brief Check if the motor is currently running.
     * @return true if motor speed > 0.
     */
    bool motorIsRunning() const override;

    /**
     * @brief Check if a fault condition is currently active.
     * @return true if fault is triggered.
     */
    bool isFaultTriggered() const override;

    /**
     * @brief Clear the fault flag.
     */
    void clearFaultFlag() override;

    /**
     * @brief Get and clear the fault flag atomically.
     * @return true if a fault was present (now cleared).
     */
    bool getAndClearFault() override;

    /**
     * @brief Set PWM frequency and resolution.
     *
     * @param resolution PWM resolution (e.g., LEDC_TIMER_10_BIT)
     * @param frequency PWM frequency in Hz (default 20kHz).
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency = 20000);

    /**
     * @brief Set PWM frequency while keeping current resolution.
     *
     * @param frequency Desired PWM frequency in Hz.
     * @return esp_err_t ESP_OK if success, ESP_ERR_INVALID_ARG if out of allowed range.
     */
    esp_err_t setPwmFrequency(uint32_t frequency);

    /**
     * @brief Configure the allowed frequency range.
     *
     * Updates internal min/max frequency values and clamps current frequency into the new range.
     *
     * @param minHz Minimum allowed frequency in Hz.
     * @param maxHz Maximum allowed frequency in Hz.
     */
    void setPwmFrequencyRange(uint32_t minHz, uint32_t maxHz);

    /**
     * @brief Get currently configured minimum PWM frequency.
     *
     * @return Minimum frequency in Hz.
     */
    uint32_t getPwmMinFrequency() const;

    /**
     * @brief Get currently configured maximum PWM frequency.
     *
     * @return Maximum frequency in Hz.
     */
    uint32_t getPwmMaxFrequency() const;

    /**
     * @brief Register a callback to be invoked when a fault is detected.
     * @param cb Callback function.
     */
    void setFaultCallback(const std::function<void()>& cb) override;

    /**
     * @brief Process a pending fault event (call from task context).
     */
    void processFaultEvent() override;

  private:
    DRV8876Config config;  ///< Config for DRV8876 motor driver

    GpioController gpio;  ///< GPIO pin management
    PwmController pwm;    ///< PWM/LEDC management
    FaultHandler fault;   ///< Fault detection and callback
    MotionRamp ramp;      ///< Speed ramping logic

    Direction motorDirection;  ///< Current motor direction
    uint8_t motorSpeed;        ///< Current speed (0–100)

    const uint8_t minMotorSpeed = 0;    ///< Min motor speed
    const uint8_t maxMotorSpeed = 100;  ///< Max motor speed
    const uint8_t motorStop = 0;        ///< Constant for stop signal

    uint8_t previousSpeedValue = 0;    ///< Previous speed for log suppression
    bool initialized = false;          ///< DRV class initialized flag
    bool enableMotorStopedLog = true;  ///< Log suppression for stop events

    static constexpr char TAG[] = "DRV8876";  ///< Logging tag
};

}  // namespace DRV8876
}  // namespace WroRobotSoftware
