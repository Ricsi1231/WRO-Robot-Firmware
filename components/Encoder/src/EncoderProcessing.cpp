#include "EncoderProcessing.hpp"
#include "esp_log.h"
#include <inttypes.h>

namespace WroRobotSoftware {
namespace Encoder {

static const char* TAG = "ENCODER_PROCESSING";

SpeedEstimator::SpeedEstimator()
    : pulsesPerRevolution(1),
      periodUs(100000),
      blendThresholdRpm(10),
      blendBandRpm(3),
      filterConfig(),
      filterState(0.0f),
      filterInitialized(false),
      iirCoeffKeep(0.0f),
      iirCoeffNew(1.0f),
      rpmFiltered(0.0f),
      rpmRaw(0.0f) {
    ESP_LOGI(TAG, "SpeedEstimator constructor (default config)");
}

void SpeedEstimator::configure(uint16_t ppr, uint32_t rpmCalcPeriodUs, uint32_t blendThreshold, uint32_t blendBand) {
    ESP_LOGI(TAG, "Configuring SpeedEstimator (PPR=%" PRIu16 ", period=%" PRIu32 "us, threshold=%" PRIu32 ", band=%" PRIu32 ")", ppr, rpmCalcPeriodUs,
             blendThreshold, blendBand);

    if (ppr == 0) {
        pulsesPerRevolution = 1;
        ESP_LOGW(TAG, "Invalid PPR=0, using default: %" PRIu16, pulsesPerRevolution);
    } else {
        pulsesPerRevolution = ppr;
    }

    if (rpmCalcPeriodUs == 0) {
        periodUs = 100000;
        ESP_LOGW(TAG, "Invalid period=0, using default: %" PRIu32 " us", periodUs);
    } else {
        periodUs = rpmCalcPeriodUs;
    }

    if (blendThreshold == 0) {
        blendThresholdRpm = 10;
        ESP_LOGW(TAG, "Invalid blend threshold=0, using default: %" PRIu32, blendThresholdRpm);
    } else {
        blendThresholdRpm = blendThreshold;
    }

    if (blendBand == 0) {
        blendBandRpm = 3;
        ESP_LOGW(TAG, "Invalid blend band=0, using default: %" PRIu32, blendBandRpm);
    } else {
        blendBandRpm = blendBand;
    }

    updateFilterCoeffs();
    ESP_LOGI(TAG, "SpeedEstimator configuration completed");
}

void SpeedEstimator::setFilter(const SpeedFilterConfig& cfg) {
    ESP_LOGI(TAG, "Setting filter config (type=%d, EMA=%.3f, IIR=%.2fHz, sample=%" PRIu32 "Hz)", static_cast<int>(cfg.filterType), cfg.emaAlpha,
             cfg.iirCutoffHz, cfg.sampleRateHz);

    filterConfig = cfg;

    if (filterConfig.emaAlpha <= 0.0f || filterConfig.emaAlpha > 1.0f) {
        filterConfig.emaAlpha = 0.3f;
        ESP_LOGW(TAG, "Invalid EMA alpha, corrected to: %.3f", filterConfig.emaAlpha);
    }

    if (filterConfig.iirCutoffHz <= 0.0f) {
        filterConfig.iirCutoffHz = 2.0f;
        ESP_LOGW(TAG, "Invalid IIR cutoff, corrected to: %.2f Hz", filterConfig.iirCutoffHz);
    }

    if (filterConfig.sampleRateHz == 0) {
        filterConfig.sampleRateHz = 10;
        ESP_LOGW(TAG, "Invalid sample rate, corrected to: %" PRIu32 " Hz", filterConfig.sampleRateHz);
    }

    updateFilterCoeffs();
    ESP_LOGI(TAG, "Filter configuration updated");
}

SpeedFilterConfig SpeedEstimator::getFilter() const {
    ESP_LOGD(TAG, "Getting filter config");
    return filterConfig;
}

void SpeedEstimator::resetFilter() {
    ESP_LOGI(TAG, "Resetting speed filter state");
    filterState = 0.0f;
    filterInitialized = false;
    rpmFiltered = 0.0f;
    rpmRaw = 0.0f;
    ESP_LOGD(TAG, "Filter state reset completed");
}

float SpeedEstimator::perMinuteFromPeriodUs(uint32_t period) {
    const float minuteUs = 60.0f * 1000000.0f;
    const float result = minuteUs / static_cast<float>(period);
    ESP_LOGV(TAG, "Period conversion: %" PRIu32 "us -> %.2f/min", period, result);
    return result;
}

void SpeedEstimator::update(int64_t tickDelta, int64_t nowUs, int64_t elapsedSinceLastEdgeUs, bool newEdgeSeen) {
    ESP_LOGV(TAG, "Speed update: delta=%" PRId64 ", elapsed=%" PRId64 "us, newEdge=%s", tickDelta, elapsedSinceLastEdgeUs, newEdgeSeen ? "true" : "false");

    const float perMinute = perMinuteFromPeriodUs(periodUs);
    const float rpmCount = (static_cast<float>(tickDelta) / static_cast<float>(pulsesPerRevolution)) * perMinute;

    float rpmPeriod = 0.0f;
    if (newEdgeSeen && elapsedSinceLastEdgeUs > 0) {
        const float edgesPerMinute = (60.0f * 1000000.0f) / static_cast<float>(elapsedSinceLastEdgeUs);
        const float sign = (tickDelta < 0) ? -1.0f : 1.0f;
        rpmPeriod = (edgesPerMinute / static_cast<float>(pulsesPerRevolution)) * sign;
        ESP_LOGV(TAG, "Period-based RPM: %.2f (edges/min=%.1f, sign=%.0f)", rpmPeriod, edgesPerMinute, sign);
    } else {
        rpmPeriod = 0.0f;
        ESP_LOGV(TAG, "No new edge or invalid elapsed time, period RPM = 0");
    }

    const float absRpmCount = fabsf(rpmCount);
    const float center = static_cast<float>(blendThresholdRpm);
    const float band = static_cast<float>(blendBandRpm);
    float weightCount = 0.0f;

    if (absRpmCount <= (center - band)) {
        weightCount = 0.0f;
    } else if (absRpmCount >= (center + band)) {
        weightCount = 1.0f;
    } else {
        const float t = (absRpmCount - (center - band)) / (2.0f * band);
        if (t < 0.0f) {
            weightCount = 0.0f;
        } else if (t > 1.0f) {
            weightCount = 1.0f;
        } else {
            weightCount = t;
        }
    }

    const float hybridRpm = (1.0f - weightCount) * rpmPeriod + weightCount * rpmCount;

    ESP_LOGD(TAG, "RPM calculation: count=%.2f, period=%.2f, weight=%.3f, hybrid=%.2f", rpmCount, rpmPeriod, weightCount, hybridRpm);

    rpmRaw = hybridRpm;
    rpmFiltered = applyFilter(hybridRpm);

    ESP_LOGD(TAG, "RPM results: raw=%.2f, filtered=%.2f", rpmRaw, rpmFiltered);
}

float SpeedEstimator::rpm() const {
    ESP_LOGV(TAG, "Getting filtered RPM: %.2f", rpmFiltered);
    return rpmFiltered;
}

float SpeedEstimator::rpmUnfiltered() const {
    ESP_LOGV(TAG, "Getting raw RPM: %.2f", rpmRaw);
    return rpmRaw;
}

void SpeedEstimator::updateFilterCoeffs() {
    ESP_LOGD(TAG, "Updating filter coefficients (type=%d)", static_cast<int>(filterConfig.filterType));

    if (filterConfig.filterType == SpeedFilterType::SIMPLE_IIR) {
        const float twoPiFcOverFs = 2.0f * static_cast<float>(M_PI) * filterConfig.iirCutoffHz / static_cast<float>(filterConfig.sampleRateHz);
        const float a = expf(-twoPiFcOverFs);

        if (a < 0.0f) {
            iirCoeffKeep = 0.0f;
            ESP_LOGW(TAG, "IIR coefficient clamped to 0.0 (was negative)");
        } else if (a > 1.0f) {
            iirCoeffKeep = 1.0f;
            ESP_LOGW(TAG, "IIR coefficient clamped to 1.0 (was > 1.0)");
        } else {
            iirCoeffKeep = a;
        }

        iirCoeffNew = 1.0f - iirCoeffKeep;
        ESP_LOGD(TAG, "IIR coefficients: keep=%.6f, new=%.6f", iirCoeffKeep, iirCoeffNew);
    } else {
        iirCoeffKeep = 0.0f;
        iirCoeffNew = 1.0f;
        ESP_LOGD(TAG, "Non-IIR filter: coefficients set to pass-through");
    }
}

float SpeedEstimator::applyFilter(float rawValue) {
    ESP_LOGV(TAG, "Applying filter to raw value: %.2f", rawValue);

    if (filterConfig.filterType == SpeedFilterType::NONE) {
        ESP_LOGV(TAG, "No filtering applied");
        return rawValue;
    }

    if (filterConfig.filterType == SpeedFilterType::EMA) {
        if (!filterInitialized) {
            filterState = rawValue;
            filterInitialized = true;
            ESP_LOGD(TAG, "EMA filter initialized with: %.2f", rawValue);
            return rawValue;
        }

        const float prevState = filterState;
        filterState = filterConfig.emaAlpha * rawValue + (1.0f - filterConfig.emaAlpha) * filterState;
        ESP_LOGV(TAG, "EMA filter: prev=%.2f, raw=%.2f, new=%.2f (alpha=%.3f)", prevState, rawValue, filterState, filterConfig.emaAlpha);
        return filterState;
    }

    if (!filterInitialized) {
        filterState = rawValue;
        filterInitialized = true;
        ESP_LOGD(TAG, "IIR filter initialized with: %.2f", rawValue);
        return rawValue;
    }

    const float prevState = filterState;
    filterState = iirCoeffNew * rawValue + iirCoeffKeep * filterState;
    ESP_LOGV(TAG, "IIR filter: prev=%.2f, raw=%.2f, new=%.2f (coeffs: %.3f/%.3f)", prevState, rawValue, filterState, iirCoeffNew, iirCoeffKeep);
    return filterState;
}

DirectionDetector::DirectionDetector()
    : config(), initialized(false), currentDirection(motorDirection::RIGHT), lastDirectionAccum(0), lastDirectionChangeMs(0) {
    ESP_LOGI(TAG, "DirectionDetector constructor (default config)");
}

void DirectionDetector::setConfig(const DirectionConfig& cfg) {
    ESP_LOGI(TAG, "Setting direction config (hysteresis=%s, threshold=%" PRId32 ", debounce=%" PRIu32 "ms)", cfg.enableHysteresis ? "ON" : "OFF",
             cfg.hysteresisThreshold, cfg.debounceTimeMs);

    config = cfg;

    if (config.hysteresisThreshold <= 0) {
        config.hysteresisThreshold = 5;
        ESP_LOGW(TAG, "Invalid hysteresis threshold, corrected to: %" PRId32, config.hysteresisThreshold);
    }

    if (config.debounceTimeMs == 0) {
        config.debounceTimeMs = 100;
        ESP_LOGW(TAG, "Invalid debounce time, corrected to: %" PRIu32 " ms", config.debounceTimeMs);
    }

    reset();
    ESP_LOGI(TAG, "Direction detector configuration updated");
}

DirectionConfig DirectionDetector::getConfig() const {
    ESP_LOGD(TAG, "Getting direction config");
    return config;
}

void DirectionDetector::reset() {
    ESP_LOGI(TAG, "Resetting direction detector state");
    initialized = false;
    currentDirection = motorDirection::RIGHT;
    lastDirectionAccum = 0;
    lastDirectionChangeMs = 0;
    ESP_LOGD(TAG, "Direction detector reset completed");
}

motorDirection DirectionDetector::updateAndGet(int64_t accumTicksSigned, uint32_t nowMs) {
    ESP_LOGV(TAG, "Direction update: accum=%" PRId64 ", time=%" PRIu32 "ms", accumTicksSigned, nowMs);

    if (!config.enableHysteresis) {
        motorDirection rawDir = (accumTicksSigned < 0) ? motorDirection::LEFT : motorDirection::RIGHT;
        ESP_LOGV(TAG, "Hysteresis disabled, raw direction: %s", rawDir == motorDirection::LEFT ? "LEFT" : "RIGHT");
        return rawDir;
    }

    if (!initialized) {
        lastDirectionAccum = accumTicksSigned;
        currentDirection = motorDirection::RIGHT;
        lastDirectionChangeMs = nowMs;
        initialized = true;
        ESP_LOGD(TAG, "Direction detector initialized: dir=RIGHT, accum=%" PRId64 ", time=%" PRIu32, lastDirectionAccum, lastDirectionChangeMs);
        return currentDirection;
    }

    const int64_t delta = accumTicksSigned - lastDirectionAccum;
    const int64_t absDelta = llabs(delta);
    const uint32_t timeSinceLastChange = nowMs - lastDirectionChangeMs;

    const bool enoughMovement = (absDelta >= config.hysteresisThreshold);
    const bool enoughTime = (timeSinceLastChange >= config.debounceTimeMs);

    ESP_LOGV(TAG, "Direction hysteresis check: delta=%" PRId64 ", absDelta=%" PRId64 ", timeSince=%" PRIu32 "ms", delta, absDelta, timeSinceLastChange);
    ESP_LOGV(TAG, "Hysteresis conditions: movement=%s (need %" PRId32 "), time=%s (need %" PRIu32 "ms)", enoughMovement ? "OK" : "NO",
             config.hysteresisThreshold, enoughTime ? "OK" : "NO", config.debounceTimeMs);

    if (enoughMovement && enoughTime) {
        const motorDirection newDir = (delta > 0) ? motorDirection::RIGHT : motorDirection::LEFT;

        if (newDir != currentDirection) {
            ESP_LOGI(TAG, "Direction change detected: %s -> %s (delta=%" PRId64 ", time=%" PRIu32 "ms)",
                     currentDirection == motorDirection::LEFT ? "LEFT" : "RIGHT", newDir == motorDirection::LEFT ? "LEFT" : "RIGHT", delta,
                     timeSinceLastChange);

            currentDirection = newDir;
            lastDirectionChangeMs = nowMs;
        } else {
            ESP_LOGD(TAG, "Movement detected but direction unchanged: %s", currentDirection == motorDirection::LEFT ? "LEFT" : "RIGHT");
        }

        lastDirectionAccum = accumTicksSigned;
    } else {
        ESP_LOGV(TAG, "Insufficient movement or time for direction change");
    }

    ESP_LOGV(TAG, "Current direction: %s", currentDirection == motorDirection::LEFT ? "LEFT" : "RIGHT");
    return currentDirection;
}

}  // namespace Encoder
}  // namespace WroRobotSoftware