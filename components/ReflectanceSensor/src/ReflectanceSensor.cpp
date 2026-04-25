#include "ReflectanceSensor.hpp"

#include <cfloat>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"

namespace WroRobotSoftware {
namespace ReflectanceSensor {

ReflectanceSensor::ReflectanceSensor(const ReflectanceSensorConfig& config) : config(config) {}

ReflectanceSensor::~ReflectanceSensor() {
    stop();

    if (adcHandle != nullptr) {
        adc_oneshot_del_unit(adcHandle);
        adcHandle = nullptr;
    }

    if (stateMutex != nullptr) {
        vSemaphoreDelete(stateMutex);
        stateMutex = nullptr;
    }
}

esp_err_t ReflectanceSensor::init() {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (config.oversampleCount == 0) {
        ESP_LOGE(TAG, "Oversample count must be > 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (config.adcUnit == ADC_UNIT_2) {
        ESP_LOGW(TAG, "ADC_UNIT_2 may conflict with Wi-Fi on ESP32-S3");
    }

    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = config.adcUnit;

    esp_err_t ret = adc_oneshot_new_unit(&unitCfg, &adcHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chanCfg = {};
    chanCfg.atten = config.attenuation;
    chanCfg.bitwidth = ADC_BITWIDTH_12;

    ret = adc_oneshot_config_channel(adcHandle, config.adcChannel, &chanCfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adcHandle);
        adcHandle = nullptr;
        return ret;
    }

    initFilterCoeffs();

    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        adc_oneshot_del_unit(adcHandle);
        adcHandle = nullptr;
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG, "Initialized on ADC%d channel %d", config.adcUnit + 1, config.adcChannel);

    return ESP_OK;
}

esp_err_t ReflectanceSensor::start() {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (taskRunning) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    taskRunning = true;
    BaseType_t taskRet = xTaskCreate(sampleTaskEntry, "refl_sens", config.taskStackSize, this, config.taskPriority, &taskHandle);
    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sample task");
        taskRunning = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Started sampling at %lu ms interval", static_cast<unsigned long>(config.sampleIntervalMs));
    return ESP_OK;
}

esp_err_t ReflectanceSensor::stop() {
    if (!taskRunning) {
        return ESP_OK;
    }

    taskRunning = false;

    if (taskHandle != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(config.sampleIntervalMs * 2));
        taskHandle = nullptr;
    }

    ESP_LOGI(TAG, "Stopped");
    return ESP_OK;
}

int32_t ReflectanceSensor::getRawValue() const noexcept {
    int32_t value = 0;
    if (stateMutex != nullptr && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        value = lastRawValue;
        xSemaphoreGive(stateMutex);
    }
    return value;
}

float ReflectanceSensor::getFilteredValue() const noexcept {
    float value = 0.0f;
    if (stateMutex != nullptr && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        value = lastFilteredVoltage;
        xSemaphoreGive(stateMutex);
    }
    return value;
}

ReflectanceClass ReflectanceSensor::getDetectedClass() const noexcept {
    ReflectanceClass cls = ReflectanceClass::Unknown;
    if (stateMutex != nullptr && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        cls = lastDetectedClass;
        xSemaphoreGive(stateMutex);
    }
    return cls;
}

bool ReflectanceSensor::isStable() const noexcept {
    bool stable = false;
    if (stateMutex != nullptr && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stable = lastStable;
        xSemaphoreGive(stateMutex);
    }
    return stable;
}

void ReflectanceSensor::sampleTaskEntry(void* param) {
    auto* self = static_cast<ReflectanceSensor*>(param);
    self->sampleLoop();
    vTaskDelete(nullptr);
}

void ReflectanceSensor::sampleLoop() {
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (taskRunning) {
        int32_t raw = readOversampled();
        float avgVoltage = rawToVoltage(raw);
        float filtered = applyFilter(avgVoltage);
        ReflectanceClass newClass = classify(filtered);

        uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (newClass != pendingClass) {
            pendingClass = newClass;
            pendingClassStartMs = nowMs;
            classStable = false;
        } else if ((nowMs - pendingClassStartMs) >= config.classification.minStableTimeMs) {
            currentClass = pendingClass;
            classStable = true;
        }

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
            lastRawValue = raw;
            lastFilteredVoltage = filtered;
            lastDetectedClass = currentClass;
            lastStable = classStable;
            xSemaphoreGive(stateMutex);
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(config.sampleIntervalMs));
    }
}

int32_t ReflectanceSensor::readOversampled() {
    int64_t sum = 0;
    uint8_t validCount = 0;

    for (uint8_t i = 0; i < config.oversampleCount; i++) {
        int rawValue = 0;
        esp_err_t ret = adc_oneshot_read(adcHandle, config.adcChannel, &rawValue);
        if (ret == ESP_OK) {
            sum += rawValue;
            validCount++;
        }
    }

    if (validCount == 0) {
        ESP_LOGE(TAG, "All ADC reads failed");
        return 0;
    }

    return static_cast<int32_t>(sum / validCount);
}

float ReflectanceSensor::rawToVoltage(int32_t rawValue) { return static_cast<float>(rawValue) * ADC_MAX_VOLTAGE / static_cast<float>(ADC_MAX_RAW); }

void ReflectanceSensor::initFilterCoeffs() {
    if (config.filter.filterType == SignalFilterType::SimpleIir) {
        const float normalizedCutoff = 2.0f * static_cast<float>(M_PI) * config.filter.iirCutoffHz / static_cast<float>(config.filter.sampleRateHz);
        float decayFactor = expf(-normalizedCutoff);

        if (decayFactor < 0.0f) {
            decayFactor = 0.0f;
        } else if (decayFactor > 1.0f) {
            decayFactor = 1.0f;
        }

        iirCoeffKeep = decayFactor;
        iirCoeffNew = 1.0f - decayFactor;
    } else {
        iirCoeffKeep = 0.0f;
        iirCoeffNew = 1.0f;
    }
}

float ReflectanceSensor::applyFilter(float rawVoltage) {
    if (config.filter.filterType == SignalFilterType::None) {
        return rawVoltage;
    }

    if (config.filter.filterType == SignalFilterType::Ema) {
        if (!filterInitialized) {
            filterState = rawVoltage;
            filterInitialized = true;
            return rawVoltage;
        }
        filterState = config.filter.emaAlpha * rawVoltage + (1.0f - config.filter.emaAlpha) * filterState;
        return filterState;
    }

    if (!filterInitialized) {
        filterState = rawVoltage;
        filterInitialized = true;
        return rawVoltage;
    }

    filterState = iirCoeffNew * rawVoltage + iirCoeffKeep * filterState;
    return filterState;
}

ReflectanceClass ReflectanceSensor::classify(float voltage) const {
    if (voltage <= config.classification.noObjectVoltage) {
        return ReflectanceClass::Unknown;
    }

    if (voltage >= config.classification.saturationVoltage) {
        return ReflectanceClass::Unknown;
    }

    ReflectanceClass best = ReflectanceClass::Unknown;
    float bestDistance = FLT_MAX;

    for (uint8_t i = 0; i < config.classification.targetCount; i++) {
        const ClassRange& target = config.classification.targets[i];
        float min = target.minVoltage;
        float max = target.maxVoltage;

        if (target.classId == currentClass) {
            min -= config.classification.hysteresisVoltage;
            max += config.classification.hysteresisVoltage;
        }

        if (voltage >= min && voltage <= max) {
            float center = (target.minVoltage + target.maxVoltage) / 2.0f;
            float distance = fabsf(voltage - center);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = target.classId;
            }
        }
    }

    return best;
}

}  // namespace ReflectanceSensor
}  // namespace WroRobotSoftware
