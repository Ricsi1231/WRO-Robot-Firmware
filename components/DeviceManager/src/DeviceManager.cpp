#include "DeviceManager.hpp"

#include "BoardConfig.hpp"
#include "esp_log.h"

namespace WroRobotSoftware {
namespace DeviceManager {

DeviceManager::DeviceManager(const DeviceManagerConfig& config) : config(config) {}

DeviceManager::~DeviceManager() {
    taskRunning = false;

    if (mainTaskHandle != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOP_INTERVAL_MS * 2));
        mainTaskHandle = nullptr;
    }
}

esp_err_t DeviceManager::init() {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t ret = createComponents();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Component creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = initComponents();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Component initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "Initialized");
    return ESP_OK;
}

esp_err_t DeviceManager::start() {
    if (!initialized) {
        ESP_LOGE(TAG, "Cannot start: not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (started) {
        ESP_LOGW(TAG, "Already started");
        return ESP_OK;
    }

    esp_err_t ret = startComponents();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Component start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t taskRet = xTaskCreate(
        mainTaskEntry,
        "DeviceManager",
        config.mainTaskStackSize,
        this,
        config.mainTaskPriority,
        &mainTaskHandle);

    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main task");
        return ESP_ERR_NO_MEM;
    }

    taskRunning = true;
    started = true;
    ESP_LOGI(TAG, "Started");
    return ESP_OK;
}

esp_err_t DeviceManager::createComponents() {
    gpioInterruptManager = std::make_unique<GpioInterruptManager::GpioInterruptManager>(gpioIntConfig);

    auto motor = std::make_unique<DRV8876::DRV8876>(motorConfig);
    auto servo = std::make_unique<Servo::ServoManager>(servoManagerConfig, servoChannels, servoChannelCount);
    motionController = std::make_unique<MotionController::MotionController>(motionConfig, std::move(motor), std::move(servo));

    encoder = std::make_unique<Encoder::Encoder>(encoderConfig);
    reflectanceSensor = std::make_unique<ReflectanceSensor::ReflectanceSensor>(reflectanceConfig);
    pidController = std::make_unique<PID::PIDController>(pidConfig);
    pathPlanner = std::make_unique<PathPlanning::AStarSolver>(pathPlannerConfig);

    ESP_LOGI(TAG, "Components created");
    return ESP_OK;
}

esp_err_t DeviceManager::initComponents() {
    esp_err_t ret = gpioInterruptManager->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GpioInterruptManager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = motionController->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MotionController init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = encoder->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Encoder init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = reflectanceSensor->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ReflectanceSensor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t DeviceManager::startComponents() {
    esp_err_t ret = encoder->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Encoder start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = reflectanceSensor->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ReflectanceSensor start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

void DeviceManager::mainTaskEntry(void* param) {
    static_cast<DeviceManager*>(param)->mainTaskLoop();
}

void DeviceManager::mainTaskLoop() {
    while (taskRunning) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOP_INTERVAL_MS));
    }
    vTaskDelete(nullptr);
}

}  // namespace DeviceManager
}  // namespace WroRobotSoftware
