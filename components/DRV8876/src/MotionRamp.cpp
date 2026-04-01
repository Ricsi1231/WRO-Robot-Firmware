#include "MotionRamp.hpp"

#include <cmath>

namespace WroRobotSoftware {
namespace DRV8876 {

MotionRamp::MotionRamp(uint8_t rampStepPercent, uint32_t rampStepDelayMs, uint8_t minEffectivePwmPercent)
    : rampStepPercent(rampStepPercent), rampStepDelayMs(rampStepDelayMs), minEffectivePwmPercent(minEffectivePwmPercent) {}

esp_err_t MotionRamp::rampTo(uint8_t targetPercent, uint8_t currentSpeed, const std::function<esp_err_t(uint8_t)>& setSpeedFn) {
    if (targetPercent > 100) {
        targetPercent = 100;
    }

    if (targetPercent > 0 && targetPercent < minEffectivePwmPercent) {
        targetPercent = minEffectivePwmPercent;
    }

    if (currentSpeed == targetPercent) {
        return ESP_OK;
    }

    if (currentSpeed < targetPercent) {
        while (currentSpeed < targetPercent) {
            uint8_t nextSpeed = currentSpeed + rampStepPercent;
            if (nextSpeed > targetPercent) {
                nextSpeed = targetPercent;
            }

            if (nextSpeed > 0 && nextSpeed < minEffectivePwmPercent) {
                nextSpeed = minEffectivePwmPercent;
            }

            esp_err_t speedResult = setSpeedFn(nextSpeed);
            if (speedResult != ESP_OK) {
                return speedResult;
            }

            currentSpeed = nextSpeed;
            vTaskDelay(pdMS_TO_TICKS(rampStepDelayMs));
        }
    } else {
        while (currentSpeed > targetPercent) {
            uint8_t nextSpeed = 0;
            if (currentSpeed > rampStepPercent) {
                nextSpeed = static_cast<uint8_t>(currentSpeed - rampStepPercent);
            } else {
                nextSpeed = 0;
            }

            if (targetPercent > 0) {
                if (nextSpeed > 0 && nextSpeed < minEffectivePwmPercent) {
                    nextSpeed = minEffectivePwmPercent;
                }
            }

            if (nextSpeed < targetPercent) {
                nextSpeed = targetPercent;
            }

            esp_err_t speedResult = setSpeedFn(nextSpeed);
            if (speedResult != ESP_OK) {
                return speedResult;
            }

            currentSpeed = nextSpeed;
            vTaskDelay(pdMS_TO_TICKS(rampStepDelayMs));
        }
    }

    return ESP_OK;
}

esp_err_t MotionRamp::timedRamp(uint8_t targetPercent, uint32_t rampTimeMs, uint8_t currentSpeed, const std::function<esp_err_t(uint8_t)>& setSpeedFn) {
    if (targetPercent > 100) {
        targetPercent = 100;
    }

    if (targetPercent > 0 && targetPercent < minEffectivePwmPercent) {
        targetPercent = minEffectivePwmPercent;
    }

    if (currentSpeed == targetPercent) {
        return ESP_OK;
    }

    int delta = static_cast<int>(targetPercent) - static_cast<int>(currentSpeed);
    uint32_t steps = static_cast<uint32_t>(abs(delta));
    if (steps == 0) {
        return ESP_OK;
    }

    uint32_t delayPerStepMs = rampTimeMs / steps;
    if (delayPerStepMs == 0) {
        delayPerStepMs = 1;
    }

    int stepDirection = (delta > 0) ? 1 : -1;

    uint8_t nextSpeed = currentSpeed;
    while (nextSpeed != targetPercent) {
        nextSpeed = static_cast<uint8_t>(static_cast<int>(nextSpeed) + stepDirection);

        if (nextSpeed > 0 && nextSpeed < minEffectivePwmPercent) {
            nextSpeed = minEffectivePwmPercent;
        }

        esp_err_t stepResult = setSpeedFn(nextSpeed);
        if (stepResult != ESP_OK) {
            return stepResult;
        }

        vTaskDelay(pdMS_TO_TICKS(delayPerStepMs));
    }

    return ESP_OK;
}

}  // namespace DRV8876
}  // namespace WroRobotSoftware
