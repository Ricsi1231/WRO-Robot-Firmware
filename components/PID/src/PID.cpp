#include "PID.hpp"
#include "esp_timer.h"
#include "math.h"

using namespace WroRobotSoftware::PID;

PIDController::PIDController(const PidConfig& pidConfig) : config(pidConfig) { reset(); }

PIDController::~PIDController() {}

void PIDController::reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastOutput = 0.0f;
    lastDerivative = 0.0f;
    prevMeasured = 0.0f;
    lastTimeUs = 0;
    stuckStartTime = 0;
    smallErrorStartTime = 0;
    settled = true;
}

void PIDController::setParameters(float kp, float ki, float kd) {
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
}

void PIDController::getParameters(float& kp, float& ki, float& kd) {
    kp = config.kp;
    ki = config.ki;
    kd = config.kd;
}

bool PIDController::isSettled() const { return settled; }

float PIDController::getLastError() const { return lastError; }

float PIDController::getLastDerivative() const { return lastDerivative; }

float PIDController::getOutput() const { return lastOutput; }

float PIDController::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    uint64_t now = esp_timer_get_time();

    float dt = (now - lastTimeUs) / 1e6f;
    if (lastTimeUs == 0 || dt < 1e-6f) {
        lastTimeUs = now;
        lastError = error;
        prevMeasured = measured;
        lastDerivative = 0.0f;
        float output = config.kp * error;
        lastOutput = output;
        return output;
    }

    if (fabsf(error) < config.errorEpsilon) {
        error = 0.0f;
        integral = 0.0f;
        settled = true;
    } else {
        settled = false;
    }

    float rawDerivative = -(measured - prevMeasured) / dt;
    lastDerivative = config.derivativeAlpha * rawDerivative + (1.0f - config.derivativeAlpha) * lastDerivative;

    float tentativeOutput = config.kp * error + config.ki * integral + config.kd * lastDerivative;

    bool belowMax = fabsf(tentativeOutput) < config.maxOutput;
    bool correcting = (tentativeOutput * error < 0);

    if (belowMax || correcting) {
        integral += error * dt;
        if (integral > config.maxIntegral) integral = config.maxIntegral;
        if (integral < -config.maxIntegral) integral = -config.maxIntegral;
    }

    float output = config.kp * error + config.ki * integral + config.kd * lastDerivative;

    if (output > config.maxOutput) output = config.maxOutput;
    if (output < -config.maxOutput) output = -config.maxOutput;

    lastOutput = output;
    lastError = error;
    lastTimeUs = now;
    prevMeasured = measured;

    return output;
}
