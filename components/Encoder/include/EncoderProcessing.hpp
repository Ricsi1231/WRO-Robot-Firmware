/**
 * @file EncoderProcessing.hpp
 * @brief Encoder-specific DSP / measurement helpers: speed estimation, filtering, direction.
 *
 * Provides a hybrid RPM estimator (count-delta + edge-period) with optional
 * EMA/IIR filtering, plus a debounced/hysteretic direction detector. All
 * quantities in this module are referenced to the **encoder shaft** (i.e.,
 * before any external gear ratio or wiring inversion is applied).
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "IEncoder.hpp"

namespace WroRobotSoftware {
namespace Encoder {

using motorDirection = EncoderDirection;

/**
 * @enum SpeedFilterType
 * @brief Supported speed filtering modes.
 */
enum class SpeedFilterType {
    NONE,       ///< No filtering (raw hybrid RPM)
    EMA,        ///< Exponential Moving Average on RPM
    SIMPLE_IIR  ///< 1st-order low-pass IIR defined by cutoff & sample rate
};

/**
 * @struct SpeedFilterConfig
 * @brief Parameters for speed filtering.
 *
 * @details
 * - For `EMA`, use `emaAlpha` in (0,1]; higher -> more responsive.
 * - For `SIMPLE_IIR`, specify `iirCutoffHz` and `sampleRateHz` of the RPM
 *   update loop to derive coefficients internally.
 */
struct SpeedFilterConfig {
    SpeedFilterType filterType;  ///< Selected filter

    // EMA parameters
    float emaAlpha;  ///< Smoothing factor (0 < α ≤ 1)

    // Simple IIR parameters
    float iirCutoffHz;      ///< Low-pass cutoff frequency (Hz)
    uint32_t sampleRateHz;  ///< Sampling rate of RPM updates (Hz)
};

/**
 * @struct DirectionConfig
 * @brief Debounce and hysteresis for direction detection.
 *
 * @details
 * - `hysteresisThreshold`: minimum signed tick swing needed to accept a flip.
 * - `debounceTimeMs`: minimum time between accepted flips.
 * - `enableHysteresis`: if false, only debounce is applied.
 */
struct DirectionConfig {
    int32_t hysteresisThreshold;  ///< Tick delta needed to switch direction
    uint32_t debounceTimeMs;      ///< Min time between switches (ms)
    bool enableHysteresis;        ///< Enable threshold-based hysteresis
};

/**
 * @class SpeedEstimator
 * @brief Hybrid RPM estimator with optional filtering.
 *
 * @details
 * Combines two estimators:
 * 1) Count-delta over a fixed window (`periodUs`)
 * 2) Edge-period based estimate (from most recent encoder edge interval)
 *
 * The final raw RPM is blended around `blendThresholdRpm ± blendBandRpm`.
 * A post-filter (NONE/EMA/IIR) can be applied to produce `rpmFiltered`.
 *
 * Thread-safety: designed to be updated from a periodic timer context; getters
 * are read-only and can be called from tasks/ISRs as needed.
 */
class SpeedEstimator {
  public:
    /**
     * @brief Construct with defaults (no filter, zeroed state).
     */
    SpeedEstimator();

    /**
     * @brief Configure base estimator parameters.
     * @param ppr                Encoder pulses per revolution (per-channel PPR, not x4 CPR).
     * @param rpmCalcPeriodUs    Update period in microseconds (timer window).
     * @param blendThresholdRpm  Center RPM for blending count/period methods.
     * @param blendBandRpm       Half-width of blending band (RPM).
     */
    void configure(uint16_t ppr, uint32_t rpmCalcPeriodUs, uint32_t blendThresholdRpm, uint32_t blendBandRpm);

    /**
     * @brief Set speed filter configuration.
     * @param filterConfig Filter parameters (EMA or SIMPLE_IIR).
     */
    void setFilter(const SpeedFilterConfig& filterConfig);

    /**
     * @brief Get current filter configuration.
     * @return Copy of SpeedFilterConfig.
     */
    SpeedFilterConfig getFilter() const;

    /**
     * @brief Reset internal filter state (keeps current config).
     */
    void resetFilter();

    /**
     * @brief Update estimator once per timer window.
     *
     * @param tickDelta                 Signed tick count in the last window.
     * @param nowUs                     Current timestamp in microseconds.
     * @param elapsedSinceLastEdgeUs    Microseconds since last detected edge (0 if none yet).
     * @param newEdgeSeen               True if at least one edge occurred during the window.
     *
     * @note The method internally blends count-delta and period estimates
     *       and then applies the configured filter to produce `rpmFiltered`.
     */
    void update(int64_t tickDelta, int64_t nowUs, int64_t elapsedSinceLastEdgeUs, bool newEdgeSeen);

    /**
     * @brief Get filtered RPM (signed).
     * @return Filtered hybrid RPM.
     */
    float rpm() const;

    /**
     * @brief Get unfiltered hybrid RPM (signed).
     * @return Raw (pre-filter) RPM.
     */
    float rpmUnfiltered() const;

  private:
    /**
     * @brief Recompute IIR/EMA coefficients from current config.
     */
    void updateFilterCoeffs();

    /**
     * @brief Apply the configured filter to the raw RPM sample.
     * @param rawRpm New unfiltered sample.
     * @return Filter output.
     */
    float applyFilter(float rawRpm);

    // --- Configuration ---
    uint16_t pulsesPerRevolution = 0;  ///< Encoder PPR (per-channel)
    uint32_t periodUs = 0;             ///< Update period (µs)
    uint32_t blendThresholdRpm = 0;    ///< Blend center RPM
    uint32_t blendBandRpm = 0;         ///< Blend half-width (RPM)

    // --- Filter config/state ---
    SpeedFilterConfig filterConfig{SpeedFilterType::NONE, 1.0f, 0.0f, 0};
    float filterState = 0.0f;        ///< Last filter output
    bool filterInitialized = false;  ///< True after first sample
    float iirCoeffKeep = 1.0f;       ///< IIR: previous weight
    float iirCoeffNew = 0.0f;        ///< IIR: new-sample weight

    // --- Outputs (encoder shaft) ---
    float rpmFiltered = 0.0f;  ///< Filtered hybrid RPM
    float rpmRaw = 0.0f;       ///< Unfiltered hybrid RPM

  public:
    /**
     * @brief Convert period in microseconds to “per-minute” scale factor.
     * @param period Update period (µs).
     * @return 60e6 / period
     */
    static float perMinuteFromPeriodUs(uint32_t period);
};

/**
 * @class DirectionDetector
 * @brief Debounced, optional-hysteresis direction detector using accumulated ticks.
 *
 * @details
 * Accepts a direction flip only if:
 * - Enough signed tick swing is observed (hysteresis), and
 * - Enough time elapsed since the last accepted change (debounce).
 *
 * If hysteresis is disabled, only the debounce rule applies.
 */
class DirectionDetector {
  public:
    /**
     * @brief Construct with default (neutral) state.
     */
    DirectionDetector();

    /**
     * @brief Apply new debounce/hysteresis configuration.
     * @param cfg DirectionConfig with thresholds and debounce time.
     */
    void setConfig(const DirectionConfig& cfg);

    /**
     * @brief Read current configuration.
     * @return Copy of DirectionConfig.
     */
    DirectionConfig getConfig() const;

    /**
     * @brief Reset internal state to initial values.
     *
     * Clears last direction change markers and reverts to default direction.
     */
    void reset();

    /**
     * @brief Update detector with the current accumulated ticks and return stable direction.
     *
     * @param accumTicksSigned Signed accumulated ticks (post wiring sign).
     * @param nowMs            Current time in milliseconds.
     * @return Stable motorDirection after debounce/hysteresis evaluation.
     */
    motorDirection updateAndGet(int64_t accumTicksSigned, uint32_t nowMs);

  private:
    DirectionConfig config{0, 0, true};  ///< Debounce/hysteresis settings

    bool initialized = false;                                 ///< True after first update
    motorDirection currentDirection = motorDirection::RIGHT;  ///< Last accepted direction
    int64_t lastDirectionAccum = 0;                           ///< Tick anchor at last switch
    uint32_t lastDirectionChangeMs = 0;                       ///< Timestamp of last switch (ms)
};

}  // namespace Encoder
}  // namespace WroRobotSoftware
