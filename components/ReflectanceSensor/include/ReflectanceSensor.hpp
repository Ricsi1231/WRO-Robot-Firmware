/**
 * @file ReflectanceSensor.hpp
 * @brief IR analog reflectance sensor with voltage-based classification.
 *
 * Reads an IR reflective sensor via ADC, applies noise filtering (EMA/IIR),
 * and classifies the filtered voltage against calibrated ranges with hysteresis
 * for stable output. This is reflectance measurement, not true color detection.
 */

#pragma once

#include <cstdint>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "IReflectanceSensor.hpp"

namespace WroRobotSoftware {
namespace ReflectanceSensor {

/// Maximum raw ADC value (12-bit)
static constexpr int32_t ADC_MAX_RAW = 4095;

/// Maximum ADC voltage (V)
static constexpr float ADC_MAX_VOLTAGE = 3.3f;

/// Maximum number of classification targets
static constexpr uint8_t MAX_CLASS_TARGETS = 8;

/**
 * @enum SignalFilterType
 * @brief Available signal filter algorithms.
 */
enum class SignalFilterType {
    None,      ///< No filtering (pass-through)
    Ema,       ///< Exponential Moving Average
    SimpleIir  ///< 1st-order low-pass IIR filter
};

/**
 * @struct SignalFilterConfig
 * @brief Configuration for signal filtering.
 */
struct SignalFilterConfig {
    SignalFilterType filterType = SignalFilterType::Ema;  ///< Filter algorithm
    float emaAlpha = 0.3f;                                ///< EMA smoothing factor (0 < alpha <= 1)
    float iirCutoffHz = 5.0f;                             ///< IIR cutoff frequency (Hz)
    uint32_t sampleRateHz = 50;                           ///< Sampling rate for IIR coefficient calculation
};

/**
 * @struct ClassRange
 * @brief Voltage range mapping for a classification target.
 */
struct ClassRange {
    ReflectanceClass classId = ReflectanceClass::Unknown;  ///< Target class
    float minVoltage = 0.0f;                               ///< Minimum voltage for this class (V)
    float maxVoltage = 0.0f;                               ///< Maximum voltage for this class (V)
};

/**
 * @struct ClassificationConfig
 * @brief Configuration for voltage-based classification.
 */
struct ClassificationConfig {
    ClassRange targets[MAX_CLASS_TARGETS] = {};  ///< Calibrated target ranges
    uint8_t targetCount = 0;                     ///< Number of active targets
    float hysteresisVoltage = 0.05f;             ///< Range expansion for current class (V)
    uint32_t minStableTimeMs = 100;              ///< Minimum time before accepting class change (ms)
    float saturationVoltage = 3.2f;              ///< Above this voltage → Unknown
    float noObjectVoltage = 0.1f;                ///< Below this voltage → Unknown
};

/**
 * @struct ReflectanceSensorConfig
 * @brief Top-level configuration for the reflectance sensor.
 */
struct ReflectanceSensorConfig {
    adc_unit_t adcUnit = ADC_UNIT_1;            ///< ADC unit (prefer ADC_UNIT_1)
    adc_channel_t adcChannel = ADC_CHANNEL_0;   ///< ADC channel
    adc_atten_t attenuation = ADC_ATTEN_DB_12;  ///< ADC attenuation (DB_12 for 0-3.3V)
    uint8_t oversampleCount = 8;                ///< Number of ADC reads to average per sample
    SignalFilterConfig filter;                  ///< Signal filter configuration
    ClassificationConfig classification;        ///< Classification configuration
    uint32_t sampleIntervalMs = 20;             ///< Periodic task interval (ms)
    uint32_t taskStackSize = 3072;              ///< FreeRTOS task stack size (bytes)
    UBaseType_t taskPriority = 4;               ///< FreeRTOS task priority
};

/**
 * @class ReflectanceSensor
 * @brief Reads IR reflectance sensor via ADC with filtering and classification.
 *
 * A periodic FreeRTOS task samples the ADC, applies oversampling and digital
 * filtering, then classifies the result against calibrated voltage ranges.
 * Hysteresis and minimum stable time prevent classification flickering.
 */
class ReflectanceSensor : public IReflectanceSensor {
  public:
    /**
     * @brief Construct the sensor with configuration.
     * @param config Sensor configuration.
     */
    explicit ReflectanceSensor(const ReflectanceSensorConfig& config);

    /**
     * @brief Destructor. Stops task and releases ADC.
     */
    ~ReflectanceSensor() override;

    ReflectanceSensor(const ReflectanceSensor&) = delete;
    ReflectanceSensor& operator=(const ReflectanceSensor&) = delete;
    ReflectanceSensor(ReflectanceSensor&&) = delete;
    ReflectanceSensor& operator=(ReflectanceSensor&&) = delete;

    /**
     * @brief Initialize ADC hardware and filter state.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t init() override;

    /**
     * @brief Start the periodic sampling task.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t start() override;

    /**
     * @brief Stop the periodic sampling task.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t stop() override;

    /**
     * @brief Get the last raw ADC reading (0-4095).
     * @return Raw ADC count.
     */
    int32_t getRawValue() const noexcept override;

    /**
     * @brief Get the filtered voltage value.
     * @return Filtered voltage in volts.
     */
    float getFilteredValue() const noexcept override;

    /**
     * @brief Get the current classification result.
     * @return Detected reflectance class.
     */
    ReflectanceClass getDetectedClass() const noexcept override;

    /**
     * @brief Check if the classification has been stable for the minimum time.
     * @return true if stable.
     */
    bool isStable() const noexcept override;

  private:
    /**
     * @brief FreeRTOS task entry point.
     * @param param Pointer to ReflectanceSensor instance.
     */
    static void sampleTaskEntry(void* param);

    /**
     * @brief Main sampling loop running inside the FreeRTOS task.
     */
    void sampleLoop();

    /**
     * @brief Read ADC with oversampling (average of N reads).
     * @return Averaged raw ADC value.
     */
    int32_t readOversampled();

    /**
     * @brief Convert raw ADC value to voltage.
     * @param rawValue Raw ADC count (0-4095).
     * @return Voltage in volts.
     */
    static float rawToVoltage(int32_t rawValue);

    /**
     * @brief Compute IIR filter coefficients from config.
     */
    void initFilterCoeffs();

    /**
     * @brief Apply the configured filter to a raw voltage.
     * @param rawVoltage Input voltage.
     * @return Filtered voltage.
     */
    float applyFilter(float rawVoltage);

    /**
     * @brief Classify a filtered voltage against calibrated ranges.
     * @param voltage Filtered voltage value.
     * @return Matching reflectance class.
     */
    ReflectanceClass classify(float voltage) const;

    /// Timeout for mutex acquisition in getters (ms)
    static constexpr uint32_t MUTEX_TIMEOUT_MS = 5;

    static constexpr const char* TAG = "ReflectanceSensor";

    ReflectanceSensorConfig config;  ///< Sensor configuration

    adc_oneshot_unit_handle_t adcHandle = nullptr;  ///< ADC unit handle

    float filterState = 0.0f;        ///< Filter accumulator
    bool filterInitialized = false;  ///< Filter first-sample flag
    float iirCoeffKeep = 0.0f;       ///< IIR "keep" coefficient
    float iirCoeffNew = 1.0f;        ///< IIR "new" coefficient

    ReflectanceClass currentClass = ReflectanceClass::Unknown;  ///< Accepted class
    ReflectanceClass pendingClass = ReflectanceClass::Unknown;  ///< Candidate class
    uint32_t pendingClassStartMs = 0;                           ///< Tick when pending class started
    bool classStable = false;                                   ///< Whether current class is stable

    int32_t lastRawValue = 0;                                        ///< Shared: last raw ADC
    float lastFilteredVoltage = 0.0f;                                ///< Shared: last filtered V
    ReflectanceClass lastDetectedClass = ReflectanceClass::Unknown;  ///< Shared: last class
    bool lastStable = false;                                         ///< Shared: stability flag

    TaskHandle_t taskHandle = nullptr;       ///< FreeRTOS task handle
    SemaphoreHandle_t stateMutex = nullptr;  ///< Protects shared state
    bool initialized = false;                ///< Init completed
    bool taskRunning = false;                ///< Task loop flag
};

}  // namespace ReflectanceSensor
}  // namespace WroRobotSoftware
