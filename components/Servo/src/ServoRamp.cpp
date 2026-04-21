#include "ServoRamp.hpp"

#include <cmath>

namespace WroRobotSoftware {
namespace Servo {

ServoRamp::ServoRamp() = default;

ServoRamp::~ServoRamp() = default;

void ServoRamp::startRamp(float fromAngle, float toAngle,
                          float maxSpeedDegPerSec) {
  targetAngleDeg = toAngle;
  speedDegPerSec = std::abs(maxSpeedDegPerSec);
  direction = (toAngle >= fromAngle) ? 1.0f : -1.0f;
  active = true;
}

void ServoRamp::startTimedRamp(float fromAngle, float toAngle,
                               uint32_t durationMs) {
  targetAngleDeg = toAngle;

  uint32_t safeDuration =
      (durationMs < MIN_DURATION_MS) ? MIN_DURATION_MS : durationMs;
  float distanceDeg = std::abs(toAngle - fromAngle);
  float durationSec = static_cast<float>(safeDuration) / 1000.0f;

  speedDegPerSec = distanceDeg / durationSec;
  direction = (toAngle >= fromAngle) ? 1.0f : -1.0f;
  active = true;
}

float ServoRamp::step(float currentAngle, float deltaTimeSec) const {
  if (!active) {
    return currentAngle;
  }

  float deltaDeg = speedDegPerSec * deltaTimeSec;
  float nextAngle = currentAngle + direction * deltaDeg;


  if (direction > 0.0f && nextAngle > targetAngleDeg) {
    nextAngle = targetAngleDeg;
  } else if (direction < 0.0f && nextAngle < targetAngleDeg) {
    nextAngle = targetAngleDeg;
  }

  return nextAngle;
}

bool ServoRamp::isComplete(float currentAngle) const {
  if (!active) {
    return true;
  }
  return std::abs(currentAngle - targetAngleDeg) < ANGLE_TOLERANCE_DEG;
}

void ServoRamp::cancel() {
  active = false;
}

float ServoRamp::getTarget() const {
  return targetAngleDeg;
}

bool ServoRamp::isActive() const {
  return active;
}

}  // namespace Servo
}  // namespace WroRobotSoftware
