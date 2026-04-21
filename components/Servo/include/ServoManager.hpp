/**
 * @file ServoManager.hpp
 * @brief Multi-servo orchestrator with FreeRTOS task and command queue.
 *
 * Manages multiple servo channels with non-blocking smooth movement via a
 * periodic FreeRTOS update task. Commands are submitted through a queue for
 * thread-safe operation.
 */

#pragma once

#include <array>
#include <cstdint>

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "IServoDriver.hpp"
#include "ServoChannel.hpp"
#include "ServoRamp.hpp"

namespace WroRobotSoftware {
namespace Servo {

/**
 * @struct ServoManagerConfig
 * @brief Global configuration for the servo manager.
 */
struct ServoManagerConfig {
  ledc_timer_t ledcTimer =
      LEDC_TIMER_1;  ///< LEDC timer (MUST NOT be TIMER_0, used by DRV8876)
  ledc_timer_bit_t resolution =
      LEDC_TIMER_14_BIT;             ///< 14-bit for servo precision
  uint32_t frequencyHz = 50;         ///< Servo PWM frequency (50 Hz standard)
  uint32_t updateIntervalMs = 15;    ///< Task update period (ms)
  uint32_t taskStackSize = 4096;     ///< FreeRTOS task stack size
  UBaseType_t taskPriority = 5;      ///< FreeRTOS task priority
  uint8_t queueDepth = 16;          ///< Command queue depth
};

/**
 * @enum ServoCommandType
 * @brief Types of commands that can be sent to the servo manager.
 */
enum class ServoCommandType : uint8_t {
  SetAngle,        ///< Move to angle using slew rate limit
  SetAngleSmooth,  ///< Move to angle over a fixed duration
  Detach,          ///< Stop PWM output for a servo
  Attach           ///< Resume PWM output for a servo
};

/**
 * @struct ServoCommand
 * @brief A command for the servo manager queue.
 */
struct ServoCommand {
  ServoCommandType type;   ///< Command type
  uint8_t servoId;         ///< Target servo channel
  float angleDeg;          ///< Target angle (for SetAngle, SetAngleSmooth)
  uint32_t durationMs;     ///< Transition time (for SetAngleSmooth only)
};

/**
 * @class ServoManager
 * @brief Manages multiple servos with a FreeRTOS task for smooth movement.
 *
 * Implements IServoDriver. Commands are thread-safe via a FreeRTOS queue.
 * A periodic task steps all active ramps and updates PWM outputs.
 */
class ServoManager : public IServoDriver {
 public:
  /**
   * @brief Construct the servo manager.
   * @param config Manager configuration (timer, frequency, task params).
   * @param channelConfigs Array of per-servo configurations.
   * @param numChannels Number of servo channels to configure.
   */
  ServoManager(const ServoManagerConfig& config,
               const ServoChannelConfig* channelConfigs, uint8_t numChannels);

  /**
   * @brief Destructor. Stops the task and cleans up resources.
   */
  ~ServoManager() override;

  ServoManager(const ServoManager&) = delete;
  ServoManager& operator=(const ServoManager&) = delete;
  ServoManager(ServoManager&&) = delete;
  ServoManager& operator=(ServoManager&&) = delete;

  /**
   * @brief Initialize LEDC timer, all servo channels, and start the task.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t init() override;

  /**
   * @brief Set a servo angle using slew rate limiting.
   * @param servoId Servo channel (0-based).
   * @param angleDeg Target angle in degrees.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t setAngle(uint8_t servoId, float angleDeg) override;

  /**
   * @brief Smoothly move a servo to an angle over a given duration.
   * @param servoId Servo channel (0-based).
   * @param angleDeg Target angle in degrees.
   * @param durationMs Transition duration in milliseconds.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t setAngleSmooth(uint8_t servoId, float angleDeg,
                           uint32_t durationMs) override;

  /**
   * @brief Get the current angle of a servo.
   * @param servoId Servo channel (0-based).
   * @return Current angle in degrees (0.0 if invalid servoId).
   */
  float getAngle(uint8_t servoId) const override;

  /**
   * @brief Get the target angle a servo is moving toward.
   * @param servoId Servo channel (0-based).
   * @return Target angle in degrees (0.0 if invalid servoId).
   */
  float getTargetAngle(uint8_t servoId) const override;

  /**
   * @brief Check if a servo is currently in motion.
   * @param servoId Servo channel (0-based).
   * @return true if moving (false if invalid servoId).
   */
  bool isMoving(uint8_t servoId) const override;

  /**
   * @brief Detach a servo (stop PWM output).
   * @param servoId Servo channel (0-based).
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t detach(uint8_t servoId) override;

  /**
   * @brief Re-attach a servo (resume PWM output).
   * @param servoId Servo channel (0-based).
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t attach(uint8_t servoId) override;

  /**
   * @brief Stop the update task and detach all servos.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t stop();

  /**
   * @brief Get the number of configured servo channels.
   * @return Channel count.
   */
  uint8_t getChannelCount() const;

 private:
  /**
   * @brief FreeRTOS task entry point.
   * @param param Pointer to ServoManager instance.
   */
  static void updateTaskEntry(void* param);

  /**
   * @brief Main update loop running inside the FreeRTOS task.
   */
  void updateLoop();

  /**
   * @brief Process a single command from the queue.
   * @param cmd Command to process.
   */
  void processCommand(const ServoCommand& cmd);

  /**
   * @brief Step all active servo ramps and update PWM outputs.
   * @param deltaTimeSec Time elapsed since last update in seconds.
   */
  void updateAllServos(float deltaTimeSec);

  /**
   * @brief Validate a servo ID.
   * @param servoId Servo channel to validate.
   * @return true if valid.
   */
  bool isValidServo(uint8_t servoId) const;

  /**
   * @brief Enqueue a command to the servo task.
   * @param cmd Command to send.
   * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if queue full.
   */
  esp_err_t sendCommand(const ServoCommand& cmd);

  /// Maximum number of servo channels (ESP32-S3 LEDC limit)
  static constexpr uint8_t MAX_SERVO_CHANNELS = 8;

  /// LEDC timer used by DRV8876 — warn if user selects this
  static constexpr ledc_timer_t DRV8876_TIMER = LEDC_TIMER_0;

  static constexpr const char* TAG = "ServoManager";

  ServoManagerConfig config;    ///< Manager configuration
  uint8_t channelCount = 0;    ///< Number of active servo channels

  std::array<ServoChannel, MAX_SERVO_CHANNELS> channels;  ///< Servo channels
  std::array<ServoRamp, MAX_SERVO_CHANNELS> ramps;        ///< Per-channel ramps
  std::array<ServoChannelConfig, MAX_SERVO_CHANNELS>
      channelConfigs;           ///< Stored configs for init

  QueueHandle_t commandQueue = nullptr;      ///< Command queue
  TaskHandle_t updateTaskHandle = nullptr;   ///< FreeRTOS task handle
  SemaphoreHandle_t stateMutex = nullptr;    ///< Protects angle reads
  bool initialized = false;                  ///< Init completed
  bool taskRunning = false;                  ///< Task loop flag
};

}  // namespace Servo
}  // namespace WroRobotSoftware
