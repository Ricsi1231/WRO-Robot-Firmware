/**
 * @file Encoder.hpp
 * @brief Quadrature encoder handling class using PCNT and ESP timer on ESP32.
 *
 * Provides a high-level API for position and speed readout with hybrid estimation,
 * optional glitch filtering, direction detection, and basic health diagnostics.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "EncoderProcessing.hpp"

namespace WroRobotSoftware {
namespace Encoder {

/**
 * @struct EncoderConfig
 * @brief Configuration for the Encoder peripheral and processing pipeline.
 *
 * @details
 * - `pulsesPerRevolution` should be the encoder's electrical PPR (per-channel pulses/rev),
 *   not CPR. If the data sheet states CPR (counts per rev in x4), convert accordingly.
 * - Hybrid speed estimator blends edge-period and count-delta methods around
 *   `rpmBlendThreshold ± rpmBlendBand`.
 * - Set `filterThresholdNs` to >0 to enable PCNT glitch filtering.
 */
struct EncoderConfig {
    gpio_num_t pinA;                ///< Quadrature channel A GPIO
    gpio_num_t pinB;                ///< Quadrature channel B GPIO
    pcnt_unit_config_t unitConfig;  ///< Base PCNT unit configuration
    uint16_t pulsesPerRevolution;   ///< Encoder PPR (per-channel pulses/rev)

    uint32_t filterThresholdNs;  ///< PCNT glitch filter threshold in ns (0 disables)
    uint32_t rpmCalcPeriodUs;    ///< ESP timer period for RPM calculation (µs)
    int32_t maxRpm;              ///< Reserved for compatibility; not used in scaling
    bool enableWatchPoint;       ///< Enable PCNT watch-point wrap handling

    int watchLowLimit;   ///< Watch low limit; 0 => auto-wide
    int watchHighLimit;  ///< Watch high limit; 0 => auto-wide

    bool openCollectorInputs;  ///< true if inputs require pull-ups / OC handling

    // Hybrid speed estimator tuning
    uint32_t rpmBlendThreshold;  ///< Center RPM for estimator blending
    uint32_t rpmBlendBand;       ///< Half-width of blend band around threshold

    // Subsystem configurations (public so apps can tweak)
    SpeedFilterConfig speedFilter;  ///< Speed filter selection and params
    DirectionConfig direction;      ///< Direction detector configuration
};

/**
 * @class Encoder
 * @brief Rotary encoder reader built on ESP32 PCNT + esp_timer.
 *
 * @details
 * Features:
 * - Position in ticks and degrees (with gear ratio support).
 * - Hybrid RPM estimator (count-delta + period) with configurable blending.
 * - Direction detection with debouncing and "stable" direction fast path.
 * - Optional PCNT glitch filtering and watchpoint overflow handling.
 * - Health flags: stalled / noisy / saturated with tunable thresholds.
 *
 * Thread-safety: Methods that query state are `noexcept` and safe to call
 * from application tasks. Mutating operations guard internal state with
 * `encoderMutex` where needed.
 */
class Encoder : public IEncoder {
  public:
    /**
     * @brief Construct an Encoder with the given configuration.
     * @param config EncoderConfig with GPIOs, PCNT settings, and processing params.
     */
    explicit Encoder(EncoderConfig config);

    /**
     * @brief Destructor. Releases PCNT channels, timer, and other resources.
     */
    ~Encoder() override;

    Encoder(const Encoder&) = delete;             ///< Non-copyable
    Encoder& operator=(const Encoder&) = delete;  ///< Non-copy-assignable

    /**
     * @brief Move constructor.
     * @param other Source to move from.
     */
    Encoder(Encoder&& other) noexcept;

    /**
     * @brief Move assignment.
     * @param other Source to move from.
     * @return *this
     */
    Encoder& operator=(Encoder&& other) noexcept;

    /**
     * @brief Initialize PCNT, timer, IO, and optional watch-point logic.
     * @return ESP_OK on success or esp_err_t on failure.
     */
    esp_err_t init() override;

    /**
     * @brief Start periodic RPM calculations and enable PCNT unit.
     * @return ESP_OK on success or error code.
     */
    esp_err_t start() override;

    /**
     * @brief Stop periodic calculations and disable PCNT unit.
     * @return ESP_OK on success or error code.
     */
    esp_err_t stop() override;

    /**
     * @brief Reset accumulated position to zero.
     * @return ESP_OK on success or error code.
     */
    esp_err_t resetPosition() override;

    /**
     * @brief Electrically invert A/B interpretation (motor wiring inversion).
     * @param inverted true to invert, false to use normal polarity.
     */
    void setInverted(bool inverted) noexcept;

    /**
     * @brief Swap A and B channel interpretation at software level.
     */
    void swapAB() noexcept;

    /**
     * @brief Set mechanical gear ratio applied to position and RPM readouts.
     * @param ratio Gear ratio (>0). Values ≠1.0 scale outputs accordingly.
     */
    void setGearRatio(float ratio) noexcept override;

    /**
     * @brief Get current position in raw encoder ticks (signed).
     * @return Accumulated tick count.
     */
    int32_t getPositionTicks() const noexcept override;

    /**
     * @brief Get current mechanical position in degrees.
     * @return Position (0–360) wrapping according to tick count and ratio.
     */
    float getPositionInDegrees() const noexcept override;

    /**
     * @brief Get filtered RPM estimate (hybrid + filter).
     * @return Filtered RPM (signed by direction).
     */
    float getRpm() const noexcept override;

    /**
     * @brief Get unfiltered instantaneous RPM.
     * @return Raw RPM estimate (signed).
     */
    float getRpmUnfiltered() const noexcept override;

    /**
     * @brief Get rounded integer RPM (useful for coarse UI / logging).
     * @return Rounded RPM (signed).
     */
    int32_t getRpmRounded() const noexcept override;

    /**
     * @brief Get stable (debounced) motor direction.
     * @return EncoderDirection enum value.
     */
    EncoderDirection getMotorDirection() const noexcept override;

    /**
     * @brief Get immediate/raw direction (fast path, minimal debouncing).
     * @return EncoderDirection enum value.
     */
    EncoderDirection getMotorDirectionRaw() const noexcept override;

    /**
     * @brief Set direction detector configuration.
     * @param dirConfig Debounce / window configuration.
     */
    void setDirectionConfig(const DirectionConfig& dirConfig) noexcept;

    /**
     * @brief Get current direction detector configuration.
     * @return DirectionConfig copy.
     */
    DirectionConfig getDirectionConfig() const noexcept;

    /**
     * @brief Reset internal direction detector state.
     */
    void resetDirectionState() noexcept;

    /**
     * @brief Configure PCNT glitch filter threshold.
     * @param ns Threshold in nanoseconds (0 disables).
     * @return ESP_OK on success or error from PCNT driver.
     */
    esp_err_t setGlitchFilterNs(uint32_t ns);

    /**
     * @brief Read current glitch filter threshold.
     * @return Nanoseconds (0 if disabled).
     */
    uint32_t getGlitchFilterNs() const noexcept;

    /**
     * @brief Apply speed filter configuration.
     * @param filterConfig Filter selection and params (EMA/IIR/NONE).
     */
    void setSpeedFilter(const SpeedFilterConfig& filterConfig) noexcept;

    /**
     * @brief Get current speed filter configuration.
     * @return SpeedFilterConfig copy.
     */
    SpeedFilterConfig getSpeedFilter() const noexcept;

    /**
     * @brief Reset internal speed filter state (keeps config).
     */
    void resetSpeedFilter() noexcept;

    /**
     * @brief Health flag: true if RPM beneath stall threshold.
     * @return true if considered stalled.
     */
    bool isStalled() const noexcept override;

    /**
     * @brief Health flag: true if recent RPM stdev exceeds noise threshold.
     * @return true if considered noisy.
     */
    bool isNoisy() const noexcept override;

    /**
     * @brief Health flag: true if near PCNT saturation (tick threshold).
     * @return true if considered saturated.
     */
    bool isSaturated() const noexcept override;

    /**
     * @brief Set RPM threshold below which the encoder is considered stalled.
     * @param rpm Threshold in RPM.
     */
    void setStallThresholdRpm(float rpm) noexcept;

    /**
     * @brief Set noise threshold for recent RPM standard deviation.
     * @param rpmStdDev Threshold in RPM.
     */
    void setNoiseThreshold(float rpmStdDev) noexcept;

    /**
     * @brief Set absolute tick threshold considered "near saturation."
     * @param ticks Saturation threshold in ticks.
     */
    void setSaturationThresholdTicks(int32_t ticks) noexcept;

    /**
     * @brief Get a consistent snapshot of runtime statistics.
     * @return EncoderStats struct with counts, RPMs, and counters.
     */
    EncoderStats getStats() const noexcept override;

  private:
    /**
     * @brief Initialize PCNT unit and channels.
     */
    esp_err_t initPcnt();

    /**
     * @brief Configure PCNT glitch filter (if enabled).
     */
    esp_err_t initPcntFilter();

    /**
     * @brief Configure PCNT IO and edge actions for A/B.
     */
    esp_err_t initPcntIo();

    /**
     * @brief Create and configure esp_timer for RPM calculation.
     */
    esp_err_t initTimer();

    /**
     * @brief Configure PCNT watchpoint limits and callback (optional).
     */
    esp_err_t initWatchPoint();

    /**
     * @brief Enable the PCNT unit (start counting).
     */
    esp_err_t enablePcUnit();

    /**
     * @brief Periodic timer ISR callback for RPM/position updates.
     * @param arg Encoder* instance.
     */
    static void timerCallback(void* arg);

    /**
     * @brief PCNT watchpoint event callback (overflow/underflow handling).
     * @param unit  PCNT unit handle
     * @param edata Event data
     * @param user_ctx Encoder* instance
     * @return true if a high-priority task was woken
     */
    static bool pcntOnReachCallback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);

    /**
     * @brief Release all allocated resources (PCNT, timer, mutex).
     */
    void releaseResources();

    /**
     * @brief Compute sign for tick updates based on wiring inversion/swap.
     * @return +1 or -1
     */
    inline int directionSign() const noexcept { return ((invertedWiring ^ swappedAB) ? -1 : 1); }

    // Config and wiring
    EncoderConfig encoderConfig;  ///< Immutable user configuration
    bool invertedWiring;          ///< Invert A/B sense (electrical inversion)
    bool swappedAB;               ///< Swap A/B channels (software)
    float gearRatio;              ///< Mechanical gear ratio

    // PCNT resources
    pcnt_unit_handle_t pcntUnit = nullptr;  ///< PCNT unit handle
    pcnt_channel_handle_t chanA;            ///< PCNT channel for A
    pcnt_channel_handle_t chanB;            ///< PCNT channel for B
    esp_timer_handle_t timer;               ///< Periodic RPM calc timer

    // PCNT 64-bit accumulator
    volatile int64_t baseCount;  ///< Sum of watchpoint chunks
    int64_t lastAccumCount;      ///< Last accumulated count snapshot

    // Period estimator helper
    int64_t lastEdgeTimestampUs;  ///< Timestamp of last edge (µs)
    int64_t lastEdgeAccumCount;   ///< Accum count at last edge

    // Effective watch limits
    int lowLimit;   ///< Effective low limit (computed if auto)
    int highLimit;  ///< Effective high limit (computed if auto)

    // Flags
    bool initialized;  ///< True once init() completed

    // Health thresholds
    float stallThresholdRpm = 1.0f;            ///< Below => stalled
    float noiseThresholdRpm = 50.0f;           ///< Stdev above => noisy
    int32_t saturationThresholdTicks = 30000;  ///< Near PCNT limit => saturated

    // Diagnostics
    mutable float recentStdDev = 0.0f;  ///< Simple noise metric (rolling)

    // Cached stable direction (fast path)
    motorDirection stableDirection;  ///< Debounced direction

    // Subsystems
    SpeedEstimator speedEstimator;        ///< Hybrid speed estimator
    DirectionDetector directionDetector;  ///< Direction detector
    EncoderStats stats;                   ///< Stats snapshot

    // Concurrency
    SemaphoreHandle_t encoderMutex = nullptr;  ///< Protects shared state
};

}  // namespace Encoder
}  // namespace WroRobotSoftware