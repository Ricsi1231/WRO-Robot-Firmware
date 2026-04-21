#include "MotionController.hpp"

#include <cmath>

#include "esp_log.h"

namespace WroRobotSoftware {
namespace MotionController {

MotionController::MotionController(const MotionControllerConfig& config, std::unique_ptr<DRV8876::DRV8876> motorDriver,
                                   std::unique_ptr<Servo::ServoManager> servoManager)
    : config(config), motorDriver(std::move(motorDriver)), servoManager(std::move(servoManager)) {}

MotionController::~MotionController() = default;

esp_err_t MotionController::init() {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t ret = motorDriver->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor driver init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = servoManager->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Servo manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = applySteering(0.0f);
    if (ret != ESP_OK) {
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "Initialized");

    return ESP_OK;
}

esp_err_t MotionController::setVelocity(float velocity) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return applyVelocity(velocity);
}

esp_err_t MotionController::setSteeringAngle(float angleDeg) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return applySteering(angleDeg);
}

esp_err_t MotionController::setMotion(float velocity, float angleDeg) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = applyVelocity(velocity);
    if (ret != ESP_OK) {
        return ret;
    }

    return applySteering(angleDeg);
}

esp_err_t MotionController::stop() {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = motorDriver->stop();
    currentVelocity = 0.0f;

    if (config.centerSteeringOnStop) {
        esp_err_t steeringRet = applySteering(0.0f);
        if (ret == ESP_OK) {
            ret = steeringRet;
        }
    }

    return ret;
}

esp_err_t MotionController::emergencyStop() {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = motorDriver->brake();
    currentVelocity = 0.0f;

    return ret;
}

float MotionController::getVelocity() const { return currentVelocity; }

float MotionController::getSteeringAngle() const { return currentSteeringAngle; }

bool MotionController::isMoving() const {
    if (!initialized) {
        return false;
    }
    return motorDriver->motorIsRunning();
}

esp_err_t MotionController::applyVelocity(float velocity) {
    float clamped = clampVelocity(velocity);

    if (std::abs(clamped) < VELOCITY_DEADZONE) {
        currentVelocity = 0.0f;
        return motorDriver->stop();
    }

    MotorDirection desiredDirection = velocityToDirection(clamped);

    if (directionWouldReverse(clamped)) {
        esp_err_t ret = motorDriver->setDirectionSafe(desiredDirection);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        motorDriver->setDirection(desiredDirection);
    }

    auto speedPercent = static_cast<uint8_t>(std::abs(clamped) * VELOCITY_TO_SPEED_SCALE);

    esp_err_t ret;
    if (config.velocityRampTimeMs > 0) {
        ret = motorDriver->setSpeed(speedPercent, config.velocityRampTimeMs);
    } else {
        ret = motorDriver->setSpeed(speedPercent);
    }

    if (ret == ESP_OK) {
        currentVelocity = clamped;
    }

    return ret;
}

esp_err_t MotionController::applySteering(float angleDeg) {
    float clamped = clampSteeringAngle(angleDeg);
    float servoAngle = steeringToServoAngle(clamped);

    esp_err_t ret;
    if (config.steeringSmoothTimeMs > 0) {
        ret = servoManager->setAngleSmooth(config.steeringServoId, servoAngle, config.steeringSmoothTimeMs);
    } else {
        ret = servoManager->setAngle(config.steeringServoId, servoAngle);
    }

    if (ret == ESP_OK) {
        currentSteeringAngle = clamped;
    }

    return ret;
}

float MotionController::clampVelocity(float velocity) const {
    if (velocity < MIN_VELOCITY) {
        return MIN_VELOCITY;
    }
    if (velocity > MAX_VELOCITY) {
        return MAX_VELOCITY;
    }
    return velocity;
}

float MotionController::clampSteeringAngle(float angleDeg) const {
    if (angleDeg < -config.maxSteeringAngleDeg) {
        return -config.maxSteeringAngleDeg;
    }
    if (angleDeg > config.maxSteeringAngleDeg) {
        return config.maxSteeringAngleDeg;
    }
    return angleDeg;
}

float MotionController::steeringToServoAngle(float steeringAngleDeg) const { return config.servoCenterAngleDeg + steeringAngleDeg; }

MotorDirection MotionController::velocityToDirection(float velocity) const {
    if (velocity >= 0.0f) {
        return config.forwardDirection;
    }
    return (config.forwardDirection == MotorDirection::RIGHT) ? MotorDirection::LEFT : MotorDirection::RIGHT;
}

bool MotionController::directionWouldReverse(float newVelocity) const {
    return (currentVelocity > VELOCITY_DEADZONE && newVelocity < -VELOCITY_DEADZONE) ||
           (currentVelocity < -VELOCITY_DEADZONE && newVelocity > VELOCITY_DEADZONE);
}

}  // namespace MotionController
}  // namespace WroRobotSoftware
