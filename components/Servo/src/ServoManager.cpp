#include "ServoManager.hpp"

#include <cmath>

#include "esp_log.h"

namespace WroRobotSoftware {
namespace Servo {

ServoManager::ServoManager(const ServoManagerConfig& config,
                           const ServoChannelConfig* channelConfigs,
                           uint8_t numChannels)
    : config(config) {
  channelCount =
      (numChannels > MAX_SERVO_CHANNELS) ? MAX_SERVO_CHANNELS : numChannels;

  for (uint8_t i = 0; i < channelCount; i++) {
    this->channelConfigs[i] = channelConfigs[i];
  }
}

ServoManager::~ServoManager() {
  stop();

  if (commandQueue != nullptr) {
    vQueueDelete(commandQueue);
    commandQueue = nullptr;
  }

  if (stateMutex != nullptr) {
    vSemaphoreDelete(stateMutex);
    stateMutex = nullptr;
  }
}

esp_err_t ServoManager::init() {
  if (initialized) {
    ESP_LOGW(TAG, "Already initialized");
    return ESP_OK;
  }


  if (config.ledcTimer == DRV8876_TIMER) {
    ESP_LOGW(TAG,
             "Using LEDC_TIMER_0 which conflicts with DRV8876! "
             "Consider using LEDC_TIMER_1 or higher.");
  }


  ledc_timer_config_t timerConfig = {};
  timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  timerConfig.timer_num = config.ledcTimer;
  timerConfig.duty_resolution = config.resolution;
  timerConfig.freq_hz = config.frequencyHz;
  timerConfig.clk_cfg = LEDC_AUTO_CLK;

  esp_err_t ret = ledc_timer_config(&timerConfig);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "LEDC timer %d configured: %lu Hz, %d-bit resolution",
           config.ledcTimer, static_cast<unsigned long>(config.frequencyHz),
           static_cast<int>(config.resolution));


  for (uint8_t i = 0; i < channelCount; i++) {
    ret = channels[i].init(channelConfigs[i], config.ledcTimer,
                           config.resolution, config.frequencyHz);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init servo channel %u: %s", i,
               esp_err_to_name(ret));
      return ret;
    }
  }


  commandQueue = xQueueCreate(config.queueDepth, sizeof(ServoCommand));
  if (commandQueue == nullptr) {
    ESP_LOGE(TAG, "Failed to create command queue");
    return ESP_ERR_NO_MEM;
  }


  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == nullptr) {
    ESP_LOGE(TAG, "Failed to create state mutex");
    return ESP_ERR_NO_MEM;
  }


  taskRunning = true;
  BaseType_t taskRet = xTaskCreate(updateTaskEntry, "servo_mgr",
                                   config.taskStackSize, this,
                                   config.taskPriority, &updateTaskHandle);
  if (taskRet != pdPASS) {
    ESP_LOGE(TAG, "Failed to create update task");
    taskRunning = false;
    return ESP_ERR_NO_MEM;
  }

  initialized = true;
  ESP_LOGI(TAG, "Initialized with %u servo channels", channelCount);

  return ESP_OK;
}

esp_err_t ServoManager::setAngle(uint8_t servoId, float angleDeg) {
  if (!isValidServo(servoId)) {
    return ESP_ERR_INVALID_ARG;
  }

  ServoCommand cmd = {};
  cmd.type = ServoCommandType::SetAngle;
  cmd.servoId = servoId;
  cmd.angleDeg = angleDeg;

  return sendCommand(cmd);
}

esp_err_t ServoManager::setAngleSmooth(uint8_t servoId, float angleDeg,
                                       uint32_t durationMs) {
  if (!isValidServo(servoId)) {
    return ESP_ERR_INVALID_ARG;
  }

  ServoCommand cmd = {};
  cmd.type = ServoCommandType::SetAngleSmooth;
  cmd.servoId = servoId;
  cmd.angleDeg = angleDeg;
  cmd.durationMs = durationMs;

  return sendCommand(cmd);
}

float ServoManager::getAngle(uint8_t servoId) const {
  if (!isValidServo(servoId)) {
    return 0.0f;
  }

  float angle = 0.0f;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    angle = channels[servoId].getAngle();
    xSemaphoreGive(stateMutex);
  }

  return angle;
}

float ServoManager::getTargetAngle(uint8_t servoId) const {
  if (!isValidServo(servoId)) {
    return 0.0f;
  }

  float angle = 0.0f;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    angle = channels[servoId].getTargetAngle();
    xSemaphoreGive(stateMutex);
  }

  return angle;
}

bool ServoManager::isMoving(uint8_t servoId) const {
  if (!isValidServo(servoId)) {
    return false;
  }

  bool moving = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    moving = !channels[servoId].isAtTarget();
    xSemaphoreGive(stateMutex);
  }

  return moving;
}

esp_err_t ServoManager::detach(uint8_t servoId) {
  if (!isValidServo(servoId)) {
    return ESP_ERR_INVALID_ARG;
  }

  ServoCommand cmd = {};
  cmd.type = ServoCommandType::Detach;
  cmd.servoId = servoId;

  return sendCommand(cmd);
}

esp_err_t ServoManager::attach(uint8_t servoId) {
  if (!isValidServo(servoId)) {
    return ESP_ERR_INVALID_ARG;
  }

  ServoCommand cmd = {};
  cmd.type = ServoCommandType::Attach;
  cmd.servoId = servoId;

  return sendCommand(cmd);
}

esp_err_t ServoManager::stop() {
  if (!taskRunning) {
    return ESP_OK;
  }

  taskRunning = false;


  if (updateTaskHandle != nullptr) {
    vTaskDelay(pdMS_TO_TICKS(config.updateIntervalMs * 2));
    updateTaskHandle = nullptr;
  }


  for (uint8_t i = 0; i < channelCount; i++) {
    channels[i].detach();
  }

  ESP_LOGI(TAG, "Stopped");
  return ESP_OK;
}

uint8_t ServoManager::getChannelCount() const {
  return channelCount;
}

void ServoManager::updateTaskEntry(void* param) {
  auto* manager = static_cast<ServoManager*>(param);
  manager->updateLoop();
  vTaskDelete(nullptr);
}

void ServoManager::updateLoop() {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const float deltaTimeSec =
      static_cast<float>(config.updateIntervalMs) / 1000.0f;

  while (taskRunning) {

    ServoCommand cmd;
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
      processCommand(cmd);
    }


    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      updateAllServos(deltaTimeSec);
      xSemaphoreGive(stateMutex);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(config.updateIntervalMs));
  }
}

void ServoManager::processCommand(const ServoCommand& cmd) {
  uint8_t id = cmd.servoId;

  if (!isValidServo(id)) {
    ESP_LOGW(TAG, "Invalid servo ID in command: %u", id);
    return;
  }

  switch (cmd.type) {
    case ServoCommandType::SetAngle: {
      float currentAngle = channels[id].getAngle();
      float maxSpeed = channelConfigs[id].maxSpeedDegPerSec;
      channels[id].setTargetAngle(cmd.angleDeg);
      ramps[id].startRamp(currentAngle, cmd.angleDeg, maxSpeed);
      break;
    }

    case ServoCommandType::SetAngleSmooth: {
      float currentAngle = channels[id].getAngle();
      channels[id].setTargetAngle(cmd.angleDeg);
      ramps[id].startTimedRamp(currentAngle, cmd.angleDeg, cmd.durationMs);
      break;
    }

    case ServoCommandType::Detach:
      channels[id].detach();
      ramps[id].cancel();
      break;

    case ServoCommandType::Attach:
      channels[id].attach();
      break;
  }
}

void ServoManager::updateAllServos(float deltaTimeSec) {
  for (uint8_t i = 0; i < channelCount; i++) {
    if (!ramps[i].isActive() || !channels[i].isAttached()) {
      continue;
    }

    float currentAngle = channels[i].getAngle();
    float nextAngle = ramps[i].step(currentAngle, deltaTimeSec);

    channels[i].setAngle(nextAngle);

    if (ramps[i].isComplete(nextAngle)) {
      ramps[i].cancel();
    }
  }
}

bool ServoManager::isValidServo(uint8_t servoId) const {
  return servoId < channelCount;
}

esp_err_t ServoManager::sendCommand(const ServoCommand& cmd) {
  if (!initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Command queue full");
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

}  // namespace Servo
}  // namespace WroRobotSoftware
