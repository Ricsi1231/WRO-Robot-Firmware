#include "PwmController.hpp"

namespace WroRobotSoftware {
namespace DRV8876 {

PwmController::PwmController() {
    ledcMutex = xSemaphoreCreateMutex();
    if (ledcMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create LEDC mutex");
    }
}

PwmController::~PwmController() {
    if (ledcMutex != nullptr) {
        vSemaphoreDelete(ledcMutex);
        ledcMutex = nullptr;
    }
}

PwmController::PwmController(PwmController&& other) noexcept
    : pwmChannel(other.pwmChannel),
      resolution(other.resolution),
      frequency(other.frequency),
      minFrequency(other.minFrequency),
      maxFrequency(other.maxFrequency),
      maxDuty(other.maxDuty),
      ledcMutex(other.ledcMutex),
      ledcMutexTimeoutMs(other.ledcMutexTimeoutMs),
      initialized(other.initialized) {
    other.ledcMutex = nullptr;
    other.initialized = false;
}

PwmController& PwmController::operator=(PwmController&& other) noexcept {
    if (this != &other) {
        if (ledcMutex != nullptr) {
            vSemaphoreDelete(ledcMutex);
        }
        pwmChannel = other.pwmChannel;
        resolution = other.resolution;
        frequency = other.frequency;
        minFrequency = other.minFrequency;
        maxFrequency = other.maxFrequency;
        maxDuty = other.maxDuty;
        ledcMutex = other.ledcMutex;
        ledcMutexTimeoutMs = other.ledcMutexTimeoutMs;
        initialized = other.initialized;
        other.ledcMutex = nullptr;
        other.initialized = false;
    }
    return *this;
}

esp_err_t PwmController::init(gpio_num_t enPin, ledc_channel_t channel, ledc_timer_bit_t resolution, uint32_t frequency) {
    this->pwmChannel = channel;
    this->resolution = resolution;
    this->frequency = frequency;

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = resolution;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = frequency;
    timer.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t ret = ledc_timer_config(&timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LEDC timer set (res=%d, freq=%" PRIu32 " Hz)", resolution, frequency);

    ledc_channel_config_t channelConf = {};
    channelConf.gpio_num = enPin;
    channelConf.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConf.channel = channel;
    channelConf.timer_sel = LEDC_TIMER_0;
    channelConf.duty = 0;
    channelConf.hpoint = 0;
    channelConf.flags.output_invert = false;

    ret = ledc_channel_config(&channelConf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LEDC channel configured on GPIO %d", enPin);

    maxDuty = (1u << resolution) - 1u;
    initialized = true;
    return ESP_OK;
}

esp_err_t PwmController::setDuty(uint8_t duty) {
    if (initialized == false) {
        ESP_LOGE(TAG, "PwmController is not initialized");
        return ESP_FAIL;
    }

    if (!lockLedc(ledcMutexTimeoutMs)) {
        ESP_LOGE(TAG, "LEDC lock timeout (set duty)");
        return ESP_ERR_TIMEOUT;
    }

    uint32_t scaledDuty = (duty * ((1 << resolution) - 1)) / 255;
    esp_err_t setResult = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, scaledDuty);
    if (setResult == ESP_OK) {
        setResult = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    }

    unlockLedc();

    if (setResult != ESP_OK) {
        ESP_LOGE(TAG, "PWM duty update failed: %s", esp_err_to_name(setResult));
        return setResult;
    }

    ESP_LOGD(TAG, "PWM duty set (scaled=%" PRIu32 ")", scaledDuty);
    return ESP_OK;
}

esp_err_t PwmController::setDutyTicks(uint32_t dutyTicks) {
    const uint32_t maxDutyTicks = (1u << resolution) - 1u;
    if (dutyTicks > maxDutyTicks) {
        dutyTicks = maxDutyTicks;
    }

    esp_err_t setResult = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, dutyTicks);
    if (setResult == ESP_OK) {
        setResult = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    }

    if (setResult != ESP_OK) {
        ESP_LOGE(TAG, "PWM ticks update failed: %s", esp_err_to_name(setResult));
        return setResult;
    }

    return ESP_OK;
}

uint32_t PwmController::percentToTicks(uint8_t percent) const {
    if (percent > 100) {
        percent = 100;
    }
    const uint32_t maxDutyTicks = (1u << resolution) - 1u;
    return (maxDutyTicks * static_cast<uint32_t>(percent) + 50u) / 100u;
}

esp_err_t PwmController::setTimerConfig(ledc_timer_bit_t resolution, uint32_t frequency) {
    if (initialized == false) {
        ESP_LOGE(TAG, "PwmController is not initialized");
        return ESP_FAIL;
    }

    if (frequency < minFrequency || frequency > maxFrequency) {
        ESP_LOGW(TAG, "Frequency out of range %" PRIu32 " Hz (allowed %" PRIu32 "..%" PRIu32 ")", frequency, minFrequency, maxFrequency);
        return ESP_ERR_INVALID_ARG;
    }

    this->resolution = resolution;
    this->frequency = frequency;

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = resolution;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = frequency;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    maxDuty = (1u << resolution) - 1u;
    ESP_LOGI(TAG, "PWM timer updated (res=%d, freq=%" PRIu32 " Hz)", resolution, frequency);
    return ESP_OK;
}

esp_err_t PwmController::setFrequency(uint32_t frequency) { return setTimerConfig(resolution, frequency); }

void PwmController::setFrequencyRange(uint32_t minHz, uint32_t maxHz) {
    if (minHz == 0) {
        minHz = 1;
    }
    if (maxHz < minHz) {
        maxHz = minHz;
    }

    minFrequency = minHz;
    maxFrequency = maxHz;

    if (frequency < minFrequency) {
        frequency = minFrequency;
    }
    if (frequency > maxFrequency) {
        frequency = maxFrequency;
    }
}

uint32_t PwmController::getMinFrequency() const { return minFrequency; }

uint32_t PwmController::getMaxFrequency() const { return maxFrequency; }

ledc_timer_bit_t PwmController::getResolution() const { return resolution; }

uint32_t PwmController::getFrequency() const { return frequency; }

bool PwmController::lockLedc(uint32_t timeoutMs) {
    if (ledcMutex == nullptr) {
        return false;
    }

    TickType_t ticks;
    if (timeoutMs == portMAX_DELAY) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeoutMs);
    }

    BaseType_t takeResult = xSemaphoreTake(ledcMutex, ticks);
    return (takeResult == pdTRUE);
}

void PwmController::unlockLedc() {
    if (ledcMutex != nullptr) {
        xSemaphoreGive(ledcMutex);
    }
}

}  // namespace DRV8876
}  // namespace WroRobotSoftware
