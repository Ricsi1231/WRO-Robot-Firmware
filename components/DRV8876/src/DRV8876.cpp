#include "DRV8876.hpp"

namespace WroRobotSoftware {
namespace DRV8876 {

DRV8876::DRV8876(const DRV8876Config& config) : config(config) {}

DRV8876::~DRV8876() { ESP_LOGI(TAG, "Destructor called"); }

DRV8876::DRV8876(DRV8876&& other) noexcept
    : config(other.config),
      gpio(std::move(other.gpio)),
      pwm(std::move(other.pwm)),
      fault(std::move(other.fault)),
      ramp(std::move(other.ramp)),
      motorDirection(other.motorDirection),
      motorSpeed(other.motorSpeed),
      previousSpeedValue(other.previousSpeedValue),
      initialized(other.initialized),
      enableMotorStopedLog(other.enableMotorStopedLog) {
    other.initialized = false;
}

DRV8876& DRV8876::operator=(DRV8876&& other) noexcept {
    if (this != &other) {
        config = other.config;
        gpio = std::move(other.gpio);
        pwm = std::move(other.pwm);
        fault = std::move(other.fault);
        ramp = std::move(other.ramp);
        motorDirection = other.motorDirection;
        motorSpeed = other.motorSpeed;
        previousSpeedValue = other.previousSpeedValue;
        initialized = other.initialized;
        enableMotorStopedLog = other.enableMotorStopedLog;
        other.initialized = false;
    }
    return *this;
}

esp_err_t DRV8876::init() {
    ESP_LOGI(TAG, "Init start (PH=%d, EN=%d, nFAULT=%d, nSLEEP=%d, PWM_CH=%d)", config.phPin, config.enPin, config.nFault, config.nSleep, config.pwmChannel);

    if (config.minFrequency == 0) {
        config.minFrequency = 100;
    }
    if (config.maxFrequency == 0 || config.maxFrequency < config.minFrequency) {
        config.maxFrequency = 100000;
    }
    if (config.frequency < config.minFrequency) {
        config.frequency = config.minFrequency;
    }
    if (config.frequency > config.maxFrequency) {
        config.frequency = config.maxFrequency;
    }
    if (config.rampStepPercent == 0) {
        config.rampStepPercent = 5;
    }
    if (config.rampStepDelayMs == 0) {
        config.rampStepDelayMs = 5;
    }
    if (config.minEffectivePwmPercent == 0) {
        config.minEffectivePwmPercent = 3;
    }

    esp_err_t ret = gpio.init(config.phPin, config.nSleep);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = fault.init(config.nFault);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = pwm.init(config.enPin, config.pwmChannel, config.resolution, config.frequency);
    if (ret != ESP_OK) {
        return ret;
    }

    pwm.setFrequencyRange(config.minFrequency, config.maxFrequency);
    ramp = MotionRamp(config.rampStepPercent, config.rampStepDelayMs, config.minEffectivePwmPercent);

    initialized = true;

    stop();

    gpio.setSleep(true);
    ESP_LOGI(TAG, "nSLEEP set high");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Init done");
    return ESP_OK;
}

DRV8876Config DRV8876::getConfig() const { return config; }

void DRV8876::setDirection(MotorDirection direction) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return;
    }

    if (motorDirection == direction) {
        return;
    }

    motorDirection = direction;
    gpio.setDirection(static_cast<bool>(motorDirection));
    ESP_LOGI(TAG, "Direction=%s", motorDirection == MotorDirection::LEFT ? "LEFT" : "RIGHT");
}

void DRV8876::reverseDirection() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return;
    }

    motorDirection = (motorDirection == MotorDirection::LEFT) ? MotorDirection::RIGHT : MotorDirection::LEFT;
    gpio.setDirection(static_cast<bool>(motorDirection));
    ESP_LOGI(TAG, "Direction reversed -> %s", motorDirection == MotorDirection::LEFT ? "LEFT" : "RIGHT");
}

esp_err_t DRV8876::setDirectionSafe(MotorDirection newDirection) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    uint8_t originalSpeed = motorSpeed;

    auto setSpeedFn = [this](uint8_t speed) -> esp_err_t { return setSpeed(speed); };

    esp_err_t rampDownResult = ramp.rampTo(0, motorSpeed, setSpeedFn);
    if (rampDownResult != ESP_OK) {
        return rampDownResult;
    }

    setDirection(newDirection);

    if (originalSpeed > 0 && originalSpeed < config.minEffectivePwmPercent) {
        originalSpeed = config.minEffectivePwmPercent;
    }

    esp_err_t rampUpResult = ramp.rampTo(originalSpeed, motorSpeed, setSpeedFn);
    if (rampUpResult != ESP_OK) {
        return rampUpResult;
    }

    return ESP_OK;
}

esp_err_t DRV8876::reverseDirectionSafe() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    MotorDirection reversedDirection = (motorDirection == MotorDirection::LEFT) ? MotorDirection::RIGHT : MotorDirection::LEFT;
    return setDirectionSafe(reversedDirection);
}

esp_err_t DRV8876::setSpeed(uint8_t speed) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    enableMotorStopedLog = true;

    if (speed < minMotorSpeed || speed > maxMotorSpeed) {
        ESP_LOGW(TAG, "Speed out of range (%u not in [%u..%u])", speed, minMotorSpeed, maxMotorSpeed);
        return ESP_ERR_INVALID_ARG;
    }

    if (speed > 0 && speed < config.minEffectivePwmPercent) {
        speed = config.minEffectivePwmPercent;
    }

    motorSpeed = speed;
    previousSpeedValue = motorSpeed;
    const uint32_t dutyTicks = pwm.percentToTicks(motorSpeed);

    esp_err_t setDutyResult = pwm.setDutyTicks(dutyTicks);
    if (setDutyResult != ESP_OK) {
        ESP_LOGE(TAG, "setDutyTicks failed: %s", esp_err_to_name(setDutyResult));
        return setDutyResult;
    }

    const uint32_t maxDutyTicks = (1u << config.resolution) - 1u;

    if (previousSpeedValue != motorSpeed) {
        ESP_LOGI(TAG, "Speed=%u%% (duty=%" PRIu32 "/%" PRIu32 ", res=%u-bit)", motorSpeed, dutyTicks, maxDutyTicks, static_cast<unsigned>(config.resolution));
    }

    previousSpeedValue = motorSpeed;
    return ESP_OK;
}

esp_err_t DRV8876::setSpeed(uint8_t targetPercent, uint32_t rampTimeMs) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    enableMotorStopedLog = true;

    auto setSpeedFn = [this](uint8_t speed) -> esp_err_t { return setSpeed(speed); };

    return ramp.timedRamp(targetPercent, rampTimeMs, motorSpeed, setSpeedFn);
}

esp_err_t DRV8876::stop() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = 0;

    esp_err_t ret = pwm.setDuty(motorStop);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stop -> setDuty failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    if (enableMotorStopedLog) {
        ESP_LOGI(TAG, "Stopped");
        enableMotorStopedLog = false;
    }

    return ESP_OK;
}

esp_err_t DRV8876::coast() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = motorStop;
    esp_err_t ret = pwm.setDuty(motorStop);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Coast -> setDuty failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio.setLevel(config.enPin, false);
    ESP_LOGI(TAG, "Motor set to COAST (outputs disabled, free spin)");
    return ESP_OK;
}

esp_err_t DRV8876::brake() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = motorStop;
    esp_err_t ret = pwm.setDuty(motorStop);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Brake -> setDuty failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio.setDirection(false);
    gpio.setLevel(config.enPin, false);
    ESP_LOGI(TAG, "Motor set to BRAKE (outputs shorted)");
    return ESP_OK;
}

uint8_t DRV8876::getMotorSpeed() const { return motorSpeed; }

MotorDirection DRV8876::getMotorDirection() const { return motorDirection; }

bool DRV8876::motorIsRunning() const { return motorSpeed > 0; }

bool DRV8876::isFaultTriggered() const { return fault.isFaultTriggered(); }

void DRV8876::clearFaultFlag() { fault.clearFaultFlag(); }

bool DRV8876::getAndClearFault() { return fault.getAndClearFault(); }

esp_err_t DRV8876::setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    config.resolution = resolution;
    config.frequency = frequency;

    esp_err_t ret = pwm.setTimerConfig(resolution, frequency);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t pwmVal = (motorSpeed * 255) / 100;
    ret = pwm.setDuty(pwmVal);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setDuty after timer change failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PWM updated (res=%d, freq=%" PRIu32 " Hz, duty=%" PRIu32 "/255)", resolution, frequency, (uint32_t)pwmVal);
    return ESP_OK;
}

esp_err_t DRV8876::setPwmFrequency(uint32_t frequency) { return setPwmValue(config.resolution, frequency); }

void DRV8876::setPwmFrequencyRange(uint32_t minHz, uint32_t maxHz) {
    pwm.setFrequencyRange(minHz, maxHz);
    config.minFrequency = pwm.getMinFrequency();
    config.maxFrequency = pwm.getMaxFrequency();

    if (config.frequency < config.minFrequency) {
        config.frequency = config.minFrequency;
    }
    if (config.frequency > config.maxFrequency) {
        config.frequency = config.maxFrequency;
    }
}

uint32_t DRV8876::getPwmMinFrequency() const { return pwm.getMinFrequency(); }

uint32_t DRV8876::getPwmMaxFrequency() const { return pwm.getMaxFrequency(); }

void DRV8876::setFaultCallback(const std::function<void()>& cb) { fault.setFaultCallback(cb); }

void DRV8876::processFaultEvent() { fault.processFaultEvent(); }

}  // namespace DRV8876
}  // namespace WroRobotSoftware
