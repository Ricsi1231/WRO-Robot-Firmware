/**
 * @file DeviceManager.hpp
 * @brief System initializer and component owner.
 *
 * Creates, wires, initializes, and starts all system components in the correct
 * order. Owns every top-level component via std::unique_ptr. Contains no
 * business logic — only lifecycle management.
 */

#pragma once

#include <cstdint>
#include <memory>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "AStarSolver.hpp"
#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "GpioInterruptManager.hpp"
#include "MotionController.hpp"
#include "PID.hpp"
#include "ReflectanceSensor.hpp"
#include "ServoManager.hpp"

namespace WroRobotSoftware {
namespace DeviceManager {

/**
 * @struct DeviceManagerConfig
 * @brief Configuration for the DeviceManager main task.
 */
struct DeviceManagerConfig {
    uint32_t mainTaskStackSize = 8192;  ///< Main FreeRTOS task stack size (bytes)
    UBaseType_t mainTaskPriority = 5;   ///< Main FreeRTOS task priority
};

/**
 * @class DeviceManager
 * @brief System entry point — creates, initializes, and starts all components.
 *
 * Lifecycle:
 *   1. Construct with config
 *   2. Call init() — creates and initializes all components
 *   3. Call start() — starts runtime behavior and the main task
 */
class DeviceManager {
  public:
    /**
     * @brief Construct the device manager.
     * @param config Main task configuration.
     */
    explicit DeviceManager(const DeviceManagerConfig& config);

    /**
     * @brief Destructor. Stops the main task and releases all components.
     */
    ~DeviceManager();

    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;
    DeviceManager(DeviceManager&&) = delete;
    DeviceManager& operator=(DeviceManager&&) = delete;

    /**
     * @brief Create and initialize all system components.
     * @return esp_err_t ESP_OK on success, or the first error encountered.
     */
    esp_err_t init();

    /**
     * @brief Start runtime components and the main system task.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t start();

  private:
    esp_err_t createComponents();
    esp_err_t initComponents();
    esp_err_t startComponents();

    static void mainTaskEntry(void* param);
    void mainTaskLoop();

    static constexpr const char* TAG = "DeviceManager";
    static constexpr uint32_t MAIN_TASK_LOOP_INTERVAL_MS = 100;

    DeviceManagerConfig config;
    bool initialized = false;
    bool started = false;
    bool taskRunning = false;
    TaskHandle_t mainTaskHandle = nullptr;

    std::unique_ptr<GpioInterruptManager::GpioInterruptManager> gpioInterruptManager;
    std::unique_ptr<MotionController::MotionController> motionController;
    std::unique_ptr<Encoder::Encoder> encoder;
    std::unique_ptr<ReflectanceSensor::ReflectanceSensor> reflectanceSensor;
    std::unique_ptr<PID::PIDController> pidController;
    std::unique_ptr<PathPlanning::AStarSolver> pathPlanner;
};

}  // namespace DeviceManager
}  // namespace WroRobotSoftware
