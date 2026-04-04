#include "Encoder.hpp"
#include "esp_log.h"
#include <inttypes.h>
#include <math.h>

namespace WroRobotSoftware {
namespace Encoder {

static const char* TAG = "ENCODER";

static inline int clampToPcntRange(int value) {
    const int kMax = 32700;
    const int kMin = -32700;

    if (value > kMax) {
        ESP_LOGD(TAG, "Clamping value %d to max %d", value, kMax);
        return kMax;
    }
    if (value < kMin) {
        ESP_LOGD(TAG, "Clamping value %d to min %d", value, kMin);
        return kMin;
    }
    return value;
}

Encoder::Encoder(EncoderConfig config)
    : encoderConfig(config),
      invertedWiring(false),
      swappedAB(false),
      gearRatio(1.0f),
      chanA(nullptr),
      chanB(nullptr),
      timer(nullptr),
      baseCount(0),
      lastAccumCount(0),
      lastEdgeTimestampUs(0),
      lastEdgeAccumCount(0),
      lowLimit(0),
      highLimit(0),
      initialized(false),
      stableDirection(motorDirection::RIGHT),
      speedEstimator(),
      directionDetector() {
    ESP_LOGI(TAG, "Encoder constructor (pinA=%d, pinB=%d, PPR=%" PRIu32 ")", encoderConfig.pinA, encoderConfig.pinB,
             static_cast<uint32_t>(encoderConfig.pulsesPerRevolution));

    if (encoderConfig.rpmBlendThreshold == 0u) {
        encoderConfig.rpmBlendThreshold = 10u;
        ESP_LOGD(TAG, "Setting default RPM blend threshold: %" PRIu32, static_cast<uint32_t>(encoderConfig.rpmBlendThreshold));
    }
    if (encoderConfig.rpmBlendBand == 0u) {
        encoderConfig.rpmBlendBand = 3u;
        ESP_LOGD(TAG, "Setting default RPM blend band: %" PRIu32, static_cast<uint32_t>(encoderConfig.rpmBlendBand));
    }

    if (!(encoderConfig.speedFilter.emaAlpha > 0.0f && encoderConfig.speedFilter.emaAlpha <= 1.0f)) {
        encoderConfig.speedFilter.emaAlpha = 0.3f;
        ESP_LOGD(TAG, "Setting default EMA alpha: %.3f", encoderConfig.speedFilter.emaAlpha);
    }
    if (encoderConfig.speedFilter.iirCutoffHz <= 0.0f) {
        encoderConfig.speedFilter.iirCutoffHz = 2.0f;
        ESP_LOGD(TAG, "Setting default IIR cutoff: %.2f Hz", encoderConfig.speedFilter.iirCutoffHz);
    }
    if (encoderConfig.speedFilter.sampleRateHz == 0) {
        encoderConfig.speedFilter.sampleRateHz = (encoderConfig.rpmCalcPeriodUs > 0) ? static_cast<uint32_t>(1000000u / encoderConfig.rpmCalcPeriodUs) : 10u;
        ESP_LOGD(TAG, "Setting calculated sample rate: %" PRIu32 " Hz", static_cast<uint32_t>(encoderConfig.speedFilter.sampleRateHz));
    }

    if (encoderConfig.direction.hysteresisThreshold <= 0) {
        encoderConfig.direction.hysteresisThreshold = 5;
        ESP_LOGD(TAG, "Setting default direction hysteresis threshold: %" PRId32, static_cast<int32_t>(encoderConfig.direction.hysteresisThreshold));
    }
    if (encoderConfig.direction.debounceTimeMs == 0) {
        encoderConfig.direction.debounceTimeMs = 100;
        ESP_LOGD(TAG, "Setting default direction debounce time: %" PRIu32 " ms", static_cast<uint32_t>(encoderConfig.direction.debounceTimeMs));
    }

    speedEstimator.configure(encoderConfig.pulsesPerRevolution, encoderConfig.rpmCalcPeriodUs, encoderConfig.rpmBlendThreshold, encoderConfig.rpmBlendBand);
    speedEstimator.setFilter(encoderConfig.speedFilter);
    directionDetector.setConfig(encoderConfig.direction);

    ESP_LOGI(TAG, "Encoder constructor completed");
}

Encoder::~Encoder() {
    ESP_LOGI(TAG, "Encoder destructor called");
    releaseResources();
}

Encoder::Encoder(Encoder&& other) noexcept
    : encoderConfig(other.encoderConfig),
      invertedWiring(other.invertedWiring),
      swappedAB(other.swappedAB),
      gearRatio(other.gearRatio),
      pcntUnit(other.pcntUnit),
      chanA(other.chanA),
      chanB(other.chanB),
      timer(other.timer),
      baseCount(other.baseCount),
      lastAccumCount(other.lastAccumCount),
      lastEdgeTimestampUs(other.lastEdgeTimestampUs),
      lastEdgeAccumCount(other.lastEdgeAccumCount),
      lowLimit(other.lowLimit),
      highLimit(other.highLimit),
      initialized(other.initialized),
      stableDirection(other.stableDirection),
      speedEstimator(other.speedEstimator),
      directionDetector(other.directionDetector) {
    ESP_LOGI(TAG, "Encoder move constructor");
    other.pcntUnit = nullptr;
    other.chanA = nullptr;
    other.chanB = nullptr;
    other.timer = nullptr;
    other.initialized = false;
    other.baseCount = 0;
    other.lastAccumCount = 0;
    other.lastEdgeTimestampUs = 0;
    other.lastEdgeAccumCount = 0;
    other.lowLimit = 0;
    other.highLimit = 0;
    other.invertedWiring = false;
    other.swappedAB = false;
    other.gearRatio = 1.0f;
    other.stableDirection = motorDirection::RIGHT;
}

Encoder& Encoder::operator=(Encoder&& other) noexcept {
    if (this != &other) {
        ESP_LOGI(TAG, "Encoder move assignment");
        releaseResources();

        encoderConfig = other.encoderConfig;
        invertedWiring = other.invertedWiring;
        swappedAB = other.swappedAB;
        gearRatio = other.gearRatio;
        pcntUnit = other.pcntUnit;
        other.pcntUnit = nullptr;
        chanA = other.chanA;
        other.chanA = nullptr;
        chanB = other.chanB;
        other.chanB = nullptr;
        timer = other.timer;
        other.timer = nullptr;

        baseCount = other.baseCount;
        other.baseCount = 0;
        lastAccumCount = other.lastAccumCount;
        other.lastAccumCount = 0;
        lastEdgeTimestampUs = other.lastEdgeTimestampUs;
        other.lastEdgeTimestampUs = 0;
        lastEdgeAccumCount = other.lastEdgeAccumCount;
        other.lastEdgeAccumCount = 0;
        lowLimit = other.lowLimit;
        other.lowLimit = 0;
        highLimit = other.highLimit;
        other.highLimit = 0;
        initialized = other.initialized;
        other.initialized = false;
        stableDirection = other.stableDirection;
        other.stableDirection = motorDirection::RIGHT;

        speedEstimator = other.speedEstimator;
        directionDetector = other.directionDetector;
    }
    return *this;
}

esp_err_t Encoder::init() {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized, returning OK");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Init start (pinA=%d, pinB=%d, PPR=%" PRIu32 ", period=%" PRIu32 "us)", encoderConfig.pinA, encoderConfig.pinB,
             encoderConfig.pulsesPerRevolution, encoderConfig.rpmCalcPeriodUs);

    esp_err_t result = initPcnt();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "initPcnt failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = initPcntFilter();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "initPcntFilter failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = initPcntIo();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "initPcntIo failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = initWatchPoint();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "initWatchPoint failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    baseCount = 0;
    lastAccumCount = 0;
    lastEdgeAccumCount = 0;
    lastEdgeTimestampUs = esp_timer_get_time();
    stableDirection = motorDirection::RIGHT;

    ESP_LOGD(TAG, "Resetting counters and timestamps");

    speedEstimator.configure(encoderConfig.pulsesPerRevolution, encoderConfig.rpmCalcPeriodUs, encoderConfig.rpmBlendThreshold, encoderConfig.rpmBlendBand);
    speedEstimator.setFilter(encoderConfig.speedFilter);
    directionDetector.setConfig(encoderConfig.direction);

    result = initTimer();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "initTimer failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    initialized = true;
    ESP_LOGI(TAG, "Init done");
    return ESP_OK;
}

esp_err_t Encoder::start() {
    if (!initialized) {
        ESP_LOGE(TAG, "start - Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting encoder");

    esp_err_t result = enablePcUnit();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "enablePcUnit failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    if (timer == nullptr) {
        ESP_LOGW(TAG, "Timer is null, reinitializing");
        result = initTimer();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "initTimer failed: %s (0x%x)", esp_err_to_name(result), result);
            return result;
        }
    }

    const uint64_t period = (encoderConfig.rpmCalcPeriodUs > 0) ? encoderConfig.rpmCalcPeriodUs : 100000;
    ESP_LOGI(TAG, "Starting periodic timer with period %" PRIu64 " us", period);

    result = esp_timer_start_periodic(timer, period);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Encoder started successfully");
    return ESP_OK;
}

esp_err_t Encoder::stop() {
    if (!initialized) {
        ESP_LOGE(TAG, "stop - Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping encoder");

    if (timer != nullptr) {
        const esp_err_t stopRes = esp_timer_stop(timer);
        if (stopRes != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_stop failed: %s (0x%x)", esp_err_to_name(stopRes), stopRes);
            return stopRes;
        }
        ESP_LOGD(TAG, "Timer stopped");
    }

    esp_err_t result = pcnt_unit_stop(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_stop failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_unit_disable(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_disable failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Encoder stopped successfully");
    return ESP_OK;
}

esp_err_t Encoder::resetPosition() {
    if (!initialized) {
        ESP_LOGE(TAG, "resetPosition - Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting encoder position");

    baseCount = 0;
    lastAccumCount = 0;
    lastEdgeAccumCount = 0;
    lastEdgeTimestampUs = esp_timer_get_time();

    const esp_err_t result = pcnt_unit_clear_count(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_clear_count failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    directionDetector.reset();
    speedEstimator.resetFilter();

    ESP_LOGI(TAG, "Position reset completed");
    return ESP_OK;
}

void Encoder::setInverted(bool inverted) noexcept {
    invertedWiring = inverted;
    ESP_LOGI(TAG, "Encoder wiring inverted: %s", inverted ? "YES" : "NO");
}

void Encoder::swapAB() noexcept {
    swappedAB = !swappedAB;
    ESP_LOGI(TAG, "A/B channels swapped: %s", swappedAB ? "YES" : "NO");
}

void Encoder::setGearRatio(float ratio) noexcept {
    if (ratio > 0.0f) {
        gearRatio = ratio;
        ESP_LOGI(TAG, "Gear ratio set to: %.3f", gearRatio);
    } else {
        ESP_LOGW(TAG, "Invalid gear ratio %.3f, keeping current: %.3f", ratio, gearRatio);
    }
}

int32_t Encoder::getPositionTicks() const noexcept {
    if (!initialized) {
        ESP_LOGW(TAG, "getPositionTicks - Not initialized");
        return 0;
    }

    int countSnapshot = 0;
    int64_t baseBefore = 0;
    int64_t baseAfter = 0;

    do {
        baseBefore = baseCount;
        pcnt_unit_get_count(pcntUnit, &countSnapshot);
        baseAfter = baseCount;
    } while (baseBefore != baseAfter);

    int64_t total = baseBefore + static_cast<int64_t>(countSnapshot);
    total *= static_cast<int64_t>(directionSign());

    ESP_LOGD(TAG, "Position ticks: %" PRId32 " (base=%" PRId64 ", count=%d)", static_cast<int32_t>(total), baseBefore, countSnapshot);

    return static_cast<int32_t>(total);
}

float Encoder::getPositionInDegrees() const noexcept {
    if (!initialized) {
        ESP_LOGW(TAG, "getPositionInDegrees - Not initialized");
        return 0.0f;
    }

    int countSnapshot = 0;
    int64_t baseBefore = 0;
    int64_t baseAfter = 0;

    do {
        baseBefore = baseCount;
        pcnt_unit_get_count(pcntUnit, &countSnapshot);
        baseAfter = baseCount;
    } while (baseBefore != baseAfter);

    int64_t total = baseBefore + static_cast<int64_t>(countSnapshot);
    total *= static_cast<int64_t>(directionSign());

    const uint32_t cpr = static_cast<uint32_t>(encoderConfig.pulsesPerRevolution) * 4U;
    const float angleDeg = (static_cast<float>(total) * gearRatio / static_cast<float>(cpr)) * 360.0f;

    ESP_LOGD(TAG, "Position degrees: %.2f (ticks=%" PRId64 ", CPR=%" PRIu32 ", gear=%.3f)", angleDeg, total, cpr, gearRatio);

    return angleDeg;
}

float Encoder::getRpm() const noexcept {
    const float value = speedEstimator.rpm();
    const float adjustedRpm = value * static_cast<float>(directionSign()) * gearRatio;
    ESP_LOGD(TAG, "RPM: %.2f (raw=%.2f, dir=%d, gear=%.3f)", adjustedRpm, value, directionSign(), gearRatio);
    return adjustedRpm;
}

float Encoder::getRpmUnfiltered() const noexcept {
    const float value = speedEstimator.rpmUnfiltered();
    const float adjustedRpm = value * static_cast<float>(directionSign()) * gearRatio;
    ESP_LOGD(TAG, "RPM unfiltered: %.2f (raw=%.2f)", adjustedRpm, value);
    return adjustedRpm;
}

int32_t Encoder::getRpmRounded() const noexcept {
    const int32_t rounded = static_cast<int32_t>(lroundf(getRpm()));
    ESP_LOGD(TAG, "RPM rounded: %" PRId32, rounded);
    return rounded;
}

EncoderDirection Encoder::getMotorDirection() const noexcept {
    if (!initialized || !encoderConfig.direction.enableHysteresis) {
        ESP_LOGD(TAG, "Using raw direction (not init or hysteresis disabled)");
        return getMotorDirectionRaw();
    }

    int countSnapshot = 0;
    int64_t baseBefore = 0;
    int64_t baseAfter = 0;

    do {
        baseBefore = baseCount;
        pcnt_unit_get_count(pcntUnit, &countSnapshot);
        baseAfter = baseCount;
    } while (baseBefore != baseAfter);

    const int64_t signedAccum = (baseBefore + static_cast<int64_t>(countSnapshot)) * static_cast<int64_t>(directionSign());

    const uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    motorDirection dir = const_cast<DirectionDetector&>(directionDetector).updateAndGet(signedAccum, nowMs);

    ESP_LOGD(TAG, "Motor direction: %s (with hysteresis)", dir == motorDirection::LEFT ? "LEFT" : "RIGHT");
    return dir;
}

EncoderDirection Encoder::getMotorDirectionRaw() const noexcept {
    const float rpm = getRpm();
    motorDirection dir = (rpm < 0.0f) ? motorDirection::LEFT : motorDirection::RIGHT;
    ESP_LOGD(TAG, "Raw motor direction: %s (RPM=%.2f)", dir == motorDirection::LEFT ? "LEFT" : "RIGHT", rpm);
    return dir;
}

void Encoder::setDirectionConfig(const DirectionConfig& dirConfig) noexcept {
    ESP_LOGI(TAG, "Updating direction config (hysteresis=%s, threshold=%" PRId32 ", debounce=%" PRIu32 "ms)", dirConfig.enableHysteresis ? "ON" : "OFF",
             dirConfig.hysteresisThreshold, dirConfig.debounceTimeMs);

    encoderConfig.direction = dirConfig;

    if (encoderConfig.direction.hysteresisThreshold <= 0) {
        encoderConfig.direction.hysteresisThreshold = 5;
        ESP_LOGD(TAG, "Corrected hysteresis threshold to default: %" PRId32, encoderConfig.direction.hysteresisThreshold);
    }
    if (encoderConfig.direction.debounceTimeMs == 0) {
        encoderConfig.direction.debounceTimeMs = 100;
        ESP_LOGD(TAG, "Corrected debounce time to default: %" PRIu32 " ms", encoderConfig.direction.debounceTimeMs);
    }

    directionDetector.setConfig(encoderConfig.direction);
}

DirectionConfig Encoder::getDirectionConfig() const noexcept {
    ESP_LOGD(TAG, "Getting direction config");
    return encoderConfig.direction;
}

void Encoder::resetDirectionState() noexcept {
    ESP_LOGI(TAG, "Resetting direction detector state");
    directionDetector.reset();
}

void Encoder::setSpeedFilter(const SpeedFilterConfig& filterConfig) noexcept {
    ESP_LOGI(TAG, "Updating speed filter config (EMA=%.3f, IIR=%.2fHz, sample=%" PRIu32 "Hz)", filterConfig.emaAlpha, filterConfig.iirCutoffHz,
             filterConfig.sampleRateHz);

    encoderConfig.speedFilter = filterConfig;

    if (!(encoderConfig.speedFilter.emaAlpha > 0.0f && encoderConfig.speedFilter.emaAlpha <= 1.0f)) {
        encoderConfig.speedFilter.emaAlpha = 0.3f;
        ESP_LOGD(TAG, "Corrected EMA alpha to default: %.3f", encoderConfig.speedFilter.emaAlpha);
    }
    if (encoderConfig.speedFilter.iirCutoffHz <= 0.0f) {
        encoderConfig.speedFilter.iirCutoffHz = 2.0f;
        ESP_LOGD(TAG, "Corrected IIR cutoff to default: %.2f Hz", encoderConfig.speedFilter.iirCutoffHz);
    }
    if (encoderConfig.speedFilter.sampleRateHz == 0) {
        encoderConfig.speedFilter.sampleRateHz = 10;
        ESP_LOGD(TAG, "Corrected sample rate to default: %" PRIu32 " Hz", encoderConfig.speedFilter.sampleRateHz);
    }

    speedEstimator.setFilter(encoderConfig.speedFilter);
}

SpeedFilterConfig Encoder::getSpeedFilter() const noexcept {
    ESP_LOGD(TAG, "Getting speed filter config");
    return encoderConfig.speedFilter;
}

void Encoder::resetSpeedFilter() noexcept {
    ESP_LOGI(TAG, "Resetting speed filter");
    speedEstimator.resetFilter();
}

bool Encoder::isStalled() const noexcept { return fabsf(getRpm()) < stallThresholdRpm; }

bool Encoder::isNoisy() const noexcept { return recentStdDev > noiseThresholdRpm; }

bool Encoder::isSaturated() const noexcept { return (lowLimit <= -saturationThresholdTicks) || (highLimit >= saturationThresholdTicks); }

void Encoder::setStallThresholdRpm(float rpm) noexcept { stallThresholdRpm = (rpm > 0.0f) ? rpm : 1.0f; }

void Encoder::setNoiseThreshold(float rpmStdDev) noexcept { noiseThresholdRpm = (rpmStdDev > 0.0f) ? rpmStdDev : 1.0f; }

void Encoder::setSaturationThresholdTicks(int32_t ticks) noexcept { saturationThresholdTicks = (ticks > 0) ? ticks : 1000; }

EncoderStats Encoder::getStats() const noexcept { return stats; }

esp_err_t Encoder::setGlitchFilterNs(uint32_t ns) {
    ESP_LOGI(TAG, "Setting glitch filter to %" PRIu32 " ns", ns);

    if (!initialized || pcntUnit == nullptr) {
        encoderConfig.filterThresholdNs = ns;
        ESP_LOGD(TAG, "Storing filter setting for later initialization");
        return ESP_OK;
    }

    pcnt_glitch_filter_config_t filterConfig = {.max_glitch_ns = ns};
    const esp_err_t result = pcnt_unit_set_glitch_filter(pcntUnit, &filterConfig);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_set_glitch_filter failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    encoderConfig.filterThresholdNs = ns;
    ESP_LOGI(TAG, "Glitch filter updated successfully");
    return ESP_OK;
}

uint32_t Encoder::getGlitchFilterNs() const noexcept {
    ESP_LOGD(TAG, "Getting glitch filter: %" PRIu32 " ns", encoderConfig.filterThresholdNs);
    return encoderConfig.filterThresholdNs;
}

esp_err_t Encoder::initPcnt() {
    ESP_LOGI(TAG, "Initializing PCNT unit");

    pcnt_unit_config_t localUnitConfig = encoderConfig.unitConfig;

    int requestedLow = encoderConfig.watchLowLimit;
    int requestedHigh = encoderConfig.watchHighLimit;

    if (requestedLow == 0 && requestedHigh == 0) {
        requestedLow = -30000;
        requestedHigh = 30000;
        ESP_LOGD(TAG, "Using default watch limits: low=%d, high=%d", requestedLow, requestedHigh);
    }

    lowLimit = clampToPcntRange((requestedLow < 0) ? requestedLow : -1);
    highLimit = clampToPcntRange((requestedHigh > 0) ? requestedHigh : 1);

    ESP_LOGI(TAG, "PCNT limits: low=%d, high=%d", lowLimit, highLimit);

    localUnitConfig.low_limit = lowLimit;
    localUnitConfig.high_limit = highLimit;

    const esp_err_t result = pcnt_new_unit(&localUnitConfig, &pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_unit failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "PCNT unit created successfully");
    return ESP_OK;
}

esp_err_t Encoder::initPcntFilter() {
    if (encoderConfig.filterThresholdNs == 0) {
        ESP_LOGD(TAG, "No glitch filter configured, skipping");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Setting up glitch filter: %" PRIu32 " ns", encoderConfig.filterThresholdNs);

    pcnt_glitch_filter_config_t glitchFilterConfig = {.max_glitch_ns = encoderConfig.filterThresholdNs};
    const esp_err_t result = pcnt_unit_set_glitch_filter(pcntUnit, &glitchFilterConfig);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_set_glitch_filter failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Glitch filter configured successfully");
    return ESP_OK;
}

esp_err_t Encoder::initPcntIo() {
    ESP_LOGI(TAG, "Configuring GPIO pins (A=%d, B=%d, pullup=%s)", encoderConfig.pinA, encoderConfig.pinB, encoderConfig.openCollectorInputs ? "ON" : "OFF");

    gpio_config_t gpioConfig = {};
    gpioConfig.pin_bit_mask = (1ULL << encoderConfig.pinA) | (1ULL << encoderConfig.pinB);
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = encoderConfig.openCollectorInputs ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;

    esp_err_t result = gpio_config(&gpioConfig);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Creating PCNT channels");

    pcnt_chan_config_t channelAConfig = {.edge_gpio_num = encoderConfig.pinA, .level_gpio_num = encoderConfig.pinB, .flags = {}};
    pcnt_chan_config_t channelBConfig = {.edge_gpio_num = encoderConfig.pinB, .level_gpio_num = encoderConfig.pinA, .flags = {}};

    result = pcnt_new_channel(pcntUnit, &channelAConfig, &chanA);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_channel(A) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_channel_set_edge_action(chanA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "set_edge_action(A) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_channel_set_level_action(chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "set_level_action(A) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_new_channel(pcntUnit, &channelBConfig, &chanB);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_channel(B) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_channel_set_edge_action(chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "set_edge_action(B) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_channel_set_level_action(chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "set_level_action(B) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "PCNT channels configured successfully");
    return ESP_OK;
}

esp_err_t Encoder::initWatchPoint() {
    if (!encoderConfig.enableWatchPoint) {
        ESP_LOGD(TAG, "Watch points disabled, skipping");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Setting up watch points (low=%d, high=%d)", lowLimit, highLimit);

    esp_err_t result = pcnt_unit_add_watch_point(pcntUnit, lowLimit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "add_watch_point(low) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_unit_add_watch_point(pcntUnit, highLimit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "add_watch_point(high) failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    pcnt_event_callbacks_t callbacks = {};
    callbacks.on_reach = &Encoder::pcntOnReachCallback;

    result = pcnt_unit_register_event_callbacks(pcntUnit, &callbacks, this);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "register_event_callbacks failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Watch points configured successfully");
    return ESP_OK;
}

esp_err_t Encoder::enablePcUnit() {
    ESP_LOGI(TAG, "Enabling PCNT unit");

    esp_err_t result = pcnt_unit_enable(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_enable failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_unit_clear_count(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_clear_count failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    result = pcnt_unit_start(pcntUnit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_unit_start failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "PCNT unit enabled and started");
    return ESP_OK;
}

esp_err_t Encoder::initTimer() {
    if (timer != nullptr) {
        ESP_LOGD(TAG, "Timer already exists, skipping creation");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Creating RPM calculation timer");

    const esp_timer_create_args_t timerConfig = {
        .callback = &Encoder::timerCallback, .arg = this, .dispatch_method = ESP_TIMER_TASK, .name = "rpm_timer", .skip_unhandled_events = false};

    const esp_err_t result = esp_timer_create(&timerConfig, &timer);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s (0x%x)", esp_err_to_name(result), result);
        return result;
    }

    ESP_LOGI(TAG, "Timer created successfully");
    return ESP_OK;
}

void Encoder::timerCallback(void* arg) {
    Encoder* self = static_cast<Encoder*>(arg);

    int countSnapshot = 0;
    int64_t baseBefore = 0;
    int64_t baseAfter = 0;

    do {
        baseBefore = self->baseCount;
        pcnt_unit_get_count(self->pcntUnit, &countSnapshot);
        baseAfter = self->baseCount;
    } while (baseBefore != baseAfter);

    const int64_t accumNow = baseBefore + static_cast<int64_t>(countSnapshot);
    const int64_t deltaTicks = accumNow - self->lastAccumCount;
    const int64_t nowUs = esp_timer_get_time();

    bool newEdgeSeen = (deltaTicks != 0);
    int64_t elapsedSinceLastEdgeUs = 0;
    if (newEdgeSeen) {
        elapsedSinceLastEdgeUs = nowUs - self->lastEdgeTimestampUs;
        self->lastEdgeTimestampUs = nowUs;
        self->lastEdgeAccumCount = accumNow;
        ESP_LOGV(TAG, "Timer callback: new edge detected, delta=%" PRId64 ", elapsed=%" PRId64 "us", deltaTicks, elapsedSinceLastEdgeUs);
    } else {
        elapsedSinceLastEdgeUs = nowUs - self->lastEdgeTimestampUs;
        ESP_LOGV(TAG, "Timer callback: no new edge, elapsed=%" PRId64 "us since last", elapsedSinceLastEdgeUs);
    }

    if (newEdgeSeen && fabsf(deltaTicks) > (self->encoderConfig.pulsesPerRevolution * 4)) {
        self->stats.missedEdges++;
    }

    self->speedEstimator.update(deltaTicks, nowUs, elapsedSinceLastEdgeUs, newEdgeSeen);
    self->lastAccumCount = accumNow;

    self->stats.totalCounts = accumNow;
    self->stats.lastDelta = deltaTicks;
    self->stats.rpmRaw = self->speedEstimator.rpmUnfiltered();
    self->stats.rpmFiltered = self->speedEstimator.rpm();
}

bool Encoder::pcntOnReachCallback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
    (void)unit;

    const int watchVal = edata->watch_point_value;
    ESP_LOGD(TAG, "PCNT watch point reached: %d", watchVal);

    Encoder* self = static_cast<Encoder*>(user_ctx);
    if (self != nullptr) {
        self->stats.overflows++;
        self->baseCount += static_cast<int64_t>(watchVal);
        ESP_LOGV(TAG, "Updated baseCount to %" PRId64, self->baseCount);
    } else {
        ESP_LOGE(TAG, "Watch point callback: self pointer is null!");
    }

    esp_err_t clearResult = pcnt_unit_clear_count(unit);
    if (clearResult != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear count in watch callback: %s", esp_err_to_name(clearResult));
    }

    return false;
}

void Encoder::releaseResources() {
    ESP_LOGI(TAG, "Releasing encoder resources");

    if (timer != nullptr) {
        const esp_err_t stopRes = esp_timer_stop(timer);
        if (stopRes != ESP_OK) {
            ESP_LOGW(TAG, "esp_timer_stop failed: %s (0x%x)", esp_err_to_name(stopRes), stopRes);
        }
        const esp_err_t delRes = esp_timer_delete(timer);
        if (delRes != ESP_OK) {
            ESP_LOGW(TAG, "esp_timer_delete failed: %s (0x%x)", esp_err_to_name(delRes), delRes);
        }
        timer = nullptr;
        ESP_LOGD(TAG, "Timer deleted");
    }

    if (pcntUnit != nullptr) {
        const esp_err_t stopRes = pcnt_unit_stop(pcntUnit);
        if (stopRes != ESP_OK) {
            ESP_LOGW(TAG, "pcnt_unit_stop failed: %s (0x%x)", esp_err_to_name(stopRes), stopRes);
        }
        const esp_err_t disRes = pcnt_unit_disable(pcntUnit);
        if (disRes != ESP_OK) {
            ESP_LOGW(TAG, "pcnt_unit_disable failed: %s (0x%x)", esp_err_to_name(disRes), disRes);
        }
        ESP_LOGD(TAG, "PCNT unit stopped and disabled");
    }

    if (chanA != nullptr) {
        const esp_err_t delA = pcnt_del_channel(chanA);
        if (delA != ESP_OK) {
            ESP_LOGW(TAG, "pcnt_del_channel(A) failed: %s (0x%x)", esp_err_to_name(delA), delA);
        }
        chanA = nullptr;
        ESP_LOGD(TAG, "Channel A deleted");
    }

    if (chanB != nullptr) {
        const esp_err_t delB = pcnt_del_channel(chanB);
        if (delB != ESP_OK) {
            ESP_LOGW(TAG, "pcnt_del_channel(B) failed: %s (0x%x)", esp_err_to_name(delB), delB);
        }
        chanB = nullptr;
        ESP_LOGD(TAG, "Channel B deleted");
    }

    if (pcntUnit != nullptr) {
        const esp_err_t delU = pcnt_del_unit(pcntUnit);
        if (delU != ESP_OK) {
            ESP_LOGW(TAG, "pcnt_del_unit failed: %s (0x%x)", esp_err_to_name(delU), delU);
        }
        pcntUnit = nullptr;
        ESP_LOGD(TAG, "PCNT unit deleted");
    }

    initialized = false;
    invertedWiring = false;
    swappedAB = false;
    gearRatio = 1.0f;
    baseCount = 0;
    lastAccumCount = 0;
    lastEdgeTimestampUs = 0;
    lastEdgeAccumCount = 0;
    lowLimit = 0;
    highLimit = 0;
    stableDirection = motorDirection::RIGHT;

    directionDetector.reset();
    speedEstimator.resetFilter();

    ESP_LOGI(TAG, "Resource cleanup completed");
}

}  // namespace Encoder
}  // namespace WroRobotSoftware