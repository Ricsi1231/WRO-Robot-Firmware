#include "ServoChannel.hpp"

#include <cmath>

#include "esp_log.h"

namespace WroRobotSoftware {
namespace Servo {

ServoChannel::ServoChannel() = default;

ServoChannel::~ServoChannel() = default;

ServoChannel::ServoChannel(ServoChannel&& other) noexcept
    : config(other.config),
      currentAngleDeg(other.currentAngleDeg),
      targetAngleDeg(other.targetAngleDeg),
      attached(other.attached),
      initialized(other.initialized),
      totalTicks(other.totalTicks),
      periodUs(other.periodUs) {
    other.initialized = false;
    other.attached = false;
}

ServoChannel& ServoChannel::operator=(ServoChannel&& other) noexcept {
    if (this != &other) {
        config = other.config;
        currentAngleDeg = other.currentAngleDeg;
        targetAngleDeg = other.targetAngleDeg;
        attached = other.attached;
        initialized = other.initialized;
        totalTicks = other.totalTicks;
        periodUs = other.periodUs;
        other.initialized = false;
        other.attached = false;
    }
    return *this;
}

esp_err_t ServoChannel::init(const ServoChannelConfig& channelConfig, ledc_timer_t timer, ledc_timer_bit_t resolution, uint32_t frequencyHz) {
    config = channelConfig;

    if (config.minPulseUs >= config.maxPulseUs) {
        ESP_LOGE(TAG, "Invalid pulse range: min=%.1f >= max=%.1f", config.minPulseUs, config.maxPulseUs);
        return ESP_ERR_INVALID_ARG;
    }

    if (config.minAngleDeg >= config.maxAngleDeg) {
        ESP_LOGE(TAG, "Invalid angle range: min=%.1f >= max=%.1f", config.minAngleDeg, config.maxAngleDeg);
        return ESP_ERR_INVALID_ARG;
    }

    totalTicks = (1U << static_cast<uint32_t>(resolution)) - 1;
    periodUs = 1000000.0f / static_cast<float>(frequencyHz);

    ledc_channel_config_t ledcChannelConfig = {};
    ledcChannelConfig.gpio_num = config.gpioPin;
    ledcChannelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    ledcChannelConfig.channel = config.ledcChannel;
    ledcChannelConfig.timer_sel = timer;
    ledcChannelConfig.duty = 0;
    ledcChannelConfig.hpoint = 0;

    esp_err_t ret = ledc_channel_config(&ledcChannelConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;

    float safeAngle = clampAngle(config.initialAngleDeg);
    currentAngleDeg = safeAngle;
    targetAngleDeg = safeAngle;

    ret = writePwm(angleToPulseTicks(safeAngle));
    if (ret != ESP_OK) {
        return ret;
    }

    attached = true;

    ESP_LOGI(TAG, "Initialized on GPIO %d, channel %d, initial angle=%.1f°", config.gpioPin, config.ledcChannel, safeAngle);

    return ESP_OK;
}

esp_err_t ServoChannel::setAngle(float angleDeg) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    float clamped = clampAngle(angleDeg);
    currentAngleDeg = clamped;

    if (!attached) {
        return ESP_OK;
    }

    return writePwm(angleToPulseTicks(clamped));
}

float ServoChannel::getAngle() const { return currentAngleDeg; }

void ServoChannel::setTargetAngle(float angleDeg) { targetAngleDeg = clampAngle(angleDeg); }

float ServoChannel::getTargetAngle() const { return targetAngleDeg; }

esp_err_t ServoChannel::stepTowardTarget(float maxDeltaDeg) {
    float diff = targetAngleDeg - currentAngleDeg;

    if (std::abs(diff) < ANGLE_TOLERANCE_DEG) {
        return setAngle(targetAngleDeg);
    }

    float step = (diff > 0.0f) ? maxDeltaDeg : -maxDeltaDeg;

    if (std::abs(step) > std::abs(diff)) {
        step = diff;
    }

    return setAngle(currentAngleDeg + step);
}

bool ServoChannel::isAtTarget() const { return std::abs(currentAngleDeg - targetAngleDeg) < ANGLE_TOLERANCE_DEG; }

esp_err_t ServoChannel::detach() {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = writePwm(0);
    if (ret == ESP_OK) {
        attached = false;
    }
    return ret;
}

esp_err_t ServoChannel::attach() {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = writePwm(angleToPulseTicks(currentAngleDeg));
    if (ret == ESP_OK) {
        attached = true;
    }
    return ret;
}

bool ServoChannel::isAttached() const { return attached; }

ServoChannelConfig ServoChannel::getConfig() const { return config; }

float ServoChannel::clampAngle(float angleDeg) const {
    if (angleDeg < config.minAngleDeg) {
        return config.minAngleDeg;
    }
    if (angleDeg > config.maxAngleDeg) {
        return config.maxAngleDeg;
    }
    return angleDeg;
}

uint32_t ServoChannel::angleToPulseTicks(float angleDeg) const {
    float effectiveAngle = angleDeg;
    if (config.inverted) {
        effectiveAngle = config.maxAngleDeg - angleDeg + config.minAngleDeg;
    }

    float fraction = (effectiveAngle - config.minAngleDeg) / (config.maxAngleDeg - config.minAngleDeg);

    float pulseUs = config.minPulseUs + fraction * (config.maxPulseUs - config.minPulseUs);

    return static_cast<uint32_t>((pulseUs / periodUs) * static_cast<float>(totalTicks) + 0.5f);
}

esp_err_t ServoChannel::writePwm(uint32_t ticks) {
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.ledcChannel, ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.ledcChannel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

}  // namespace Servo
}  // namespace WroRobotSoftware
