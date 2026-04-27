#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"

#include "AStarSolver.hpp"
#include "DeviceManager.hpp"
#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "GpioInterruptManager.hpp"
#include "IMotorDriver.hpp"
#include "MotionController.hpp"
#include "PID.hpp"
#include "ReflectanceSensor.hpp"
#include "ServoChannel.hpp"
#include "ServoManager.hpp"

namespace WroRobotSoftware {
namespace DeviceManager {

static const GpioInterruptManager::GpioInterruptManagerConfig gpioIntConfig = {
    .eventQueueDepth = 32,
    .taskStackSize = 4096,
    .taskPriority = 5,
    .isrAllocFlags = 0,
};

static const DRV8876::DRV8876Config motorConfig = {
    .phPin = GPIO_NUM_NC,
    .enPin = GPIO_NUM_NC,
    .nFault = GPIO_NUM_NC,
    .nSleep = GPIO_NUM_NC,
    .pwmChannel = LEDC_CHANNEL_0,
    .resolution = LEDC_TIMER_10_BIT,
    .frequency = 25000,
    .minFrequency = 100,
    .maxFrequency = 100000,
    .rampStepPercent = 5,
    .rampStepDelayMs = 20,
    .minEffectivePwmPercent = 0,
};

static const Servo::ServoManagerConfig servoManagerConfig = {
    .ledcTimer = LEDC_TIMER_1,
    .resolution = LEDC_TIMER_14_BIT,
    .frequencyHz = 50,
    .updateIntervalMs = 15,
    .taskStackSize = 4096,
    .taskPriority = 5,
    .queueDepth = 16,
};

static const Servo::ServoChannelConfig servoChannels[] = {
    {
        .gpioPin = GPIO_NUM_NC,
        .ledcChannel = LEDC_CHANNEL_1,
        .minPulseUs = 1000.0f,
        .maxPulseUs = 2000.0f,
        .minAngleDeg = 0.0f,
        .maxAngleDeg = 180.0f,
        .initialAngleDeg = 90.0f,
        .inverted = false,
        .maxSpeedDegPerSec = 300.0f,
    },
};

static const uint8_t servoChannelCount = sizeof(servoChannels) / sizeof(servoChannels[0]);

static const MotionController::MotionControllerConfig motionConfig = {
    .maxSteeringAngleDeg = 30.0f,
    .servoCenterAngleDeg = 90.0f,
    .steeringServoId = 0,
    .forwardDirection = MotorDirection::RIGHT,
    .centerSteeringOnStop = true,
    .velocityRampTimeMs = 0,
    .steeringSmoothTimeMs = 0,
};

static const Encoder::EncoderConfig encoderConfig = {
    .pinA = GPIO_NUM_NC,
    .pinB = GPIO_NUM_NC,
    .unitConfig =
        {
            .low_limit = -32768,
            .high_limit = 32767,
        },
    .pulsesPerRevolution = 12,
    .filterThresholdNs = 1000,
    .rpmCalcPeriodUs = 50000,
    .maxRpm = 0,
    .enableWatchPoint = true,
    .watchLowLimit = 0,
    .watchHighLimit = 0,
    .openCollectorInputs = false,
    .rpmBlendThreshold = 60,
    .rpmBlendBand = 20,
    .speedFilter =
        {
            .filterType = Encoder::SpeedFilterType::EMA,
            .emaAlpha = 0.3f,
            .iirCutoffHz = 5.0f,
            .sampleRateHz = 20,
        },
    .direction =
        {
            .hysteresisThreshold = 2,
            .debounceTimeMs = 50,
            .enableHysteresis = true,
        },
};

static const ReflectanceSensor::ReflectanceSensorConfig reflectanceConfig = {
    .adcUnit = ADC_UNIT_1,
    .adcChannel = ADC_CHANNEL_0,
    .attenuation = ADC_ATTEN_DB_12,
    .oversampleCount = 8,
    .filter =
        {
            .filterType = ReflectanceSensor::SignalFilterType::Ema,
            .emaAlpha = 0.3f,
            .iirCutoffHz = 5.0f,
            .sampleRateHz = 50,
        },
    .classification =
        {
            .targets = {},
            .targetCount = 0,
            .hysteresisVoltage = 0.05f,
            .minStableTimeMs = 100,
            .saturationVoltage = 3.2f,
            .noObjectVoltage = 0.1f,
        },
    .sampleIntervalMs = 20,
    .taskStackSize = 3072,
    .taskPriority = 4,
};

static const PID::PidConfig pidConfig = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxOutput = 100.0f,
    .maxIntegral = 1000.0f,
    .errorEpsilon = 2.0f,
    .speedEpsilon = 7.0f,
    .errorTimeoutSec = 0.6f,
    .stuckTimeoutSec = 0.5f,
    .derivativeAlpha = 1.0f,
};

static const PathPlanning::AStarConfig pathPlannerConfig = {
    .heuristicType = PathPlanning::HeuristicType::Euclidean,
};

static const DeviceManagerConfig deviceManagerConfig = {
    .mainTaskStackSize = 8192,
    .mainTaskPriority = 5,
};

}  // namespace DeviceManager
}  // namespace WroRobotSoftware
