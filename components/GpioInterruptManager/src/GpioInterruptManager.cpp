#include "GpioInterruptManager.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace WroRobotSoftware {
namespace GpioInterruptManager {

GpioInterruptManager::GpioInterruptManager(const GpioInterruptManagerConfig& config) : config(config) {}

GpioInterruptManager::~GpioInterruptManager() {
    taskRunning = false;

    if (taskHandle != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(DISPATCH_TIMEOUT_MS * 2));
        taskHandle = nullptr;
    }

    for (size_t i = 0; i < MAX_GPIO_PINS; i++) {
        if (registrations[i].registered) {
            gpio_isr_handler_remove(static_cast<gpio_num_t>(i));
            gpio_intr_disable(static_cast<gpio_num_t>(i));
        }
    }

    if (eventQueue != nullptr) {
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
    }
}

esp_err_t GpioInterruptManager::init() {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t ret = gpio_install_isr_service(config.isrAllocFlags);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR service already installed");
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    eventQueue = xQueueCreate(config.eventQueueDepth, sizeof(GpioEvent));
    if (eventQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    taskRunning = true;
    BaseType_t taskRet = xTaskCreate(dispatchTask, "gpio_int_mgr", config.taskStackSize, this, config.taskPriority, &taskHandle);
    if (taskRet != pdPASS) {
        taskRunning = false;
        ESP_LOGE(TAG, "Failed to create dispatch task");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG, "Initialized");

    return ESP_OK;
}

esp_err_t GpioInterruptManager::registerInterrupt(gpio_num_t pin, EdgeType edge, GpioPull pull, std::function<void(gpio_num_t)> callback) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!isValidPin(pin)) {
        ESP_LOGE(TAG, "Invalid pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    auto pinIdx = static_cast<size_t>(pin);

    if (registrations[pinIdx].registered) {
        ESP_LOGE(TAG, "Pin %d already registered", pin);
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t gpioConf = {};
    gpioConf.intr_type = toGpioIntrType(edge);
    gpioConf.mode = GPIO_MODE_INPUT;
    gpioConf.pin_bit_mask = (1ULL << pin);
    gpioConf.pull_up_en = (pull == GpioPull::Up) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpioConf.pull_down_en = (pull == GpioPull::Down) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;

    esp_err_t ret = gpio_config(&gpioConf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed for pin %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    isrContexts[pinIdx].queue = eventQueue;
    isrContexts[pinIdx].pin = pin;

    registrations[pinIdx].callback = std::move(callback);
    registrations[pinIdx].edge = edge;
    registrations[pinIdx].registered = true;
    registrations[pinIdx].enabled = true;

    ret = gpio_isr_handler_add(pin, gpioISR, &isrContexts[pinIdx]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for pin %d: %s", pin, esp_err_to_name(ret));
        registrations[pinIdx].registered = false;
        registrations[pinIdx].callback = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "Registered interrupt on pin %d", pin);
    return ESP_OK;
}

esp_err_t GpioInterruptManager::unregisterInterrupt(gpio_num_t pin) {
    if (!isValidPin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    auto pinIdx = static_cast<size_t>(pin);

    if (!registrations[pinIdx].registered) {
        ESP_LOGW(TAG, "Pin %d not registered", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_isr_handler_remove(pin);
    gpio_intr_disable(pin);

    registrations[pinIdx].callback = nullptr;
    registrations[pinIdx].registered = false;
    registrations[pinIdx].enabled = false;

    ESP_LOGI(TAG, "Unregistered interrupt on pin %d", pin);
    return ESP_OK;
}

esp_err_t GpioInterruptManager::enableInterrupt(gpio_num_t pin) {
    if (!isValidPin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    auto pinIdx = static_cast<size_t>(pin);

    if (!registrations[pinIdx].registered) {
        ESP_LOGW(TAG, "Pin %d not registered", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (registrations[pinIdx].enabled) {
        return ESP_OK;
    }

    esp_err_t ret = gpio_intr_enable(pin);
    if (ret == ESP_OK) {
        registrations[pinIdx].enabled = true;
    }
    return ret;
}

esp_err_t GpioInterruptManager::disableInterrupt(gpio_num_t pin) {
    if (!isValidPin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    auto pinIdx = static_cast<size_t>(pin);

    if (!registrations[pinIdx].registered) {
        ESP_LOGW(TAG, "Pin %d not registered", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (!registrations[pinIdx].enabled) {
        return ESP_OK;
    }

    esp_err_t ret = gpio_intr_disable(pin);
    if (ret == ESP_OK) {
        registrations[pinIdx].enabled = false;
    }
    return ret;
}

void IRAM_ATTR GpioInterruptManager::gpioISR(void* arg) {
    auto* ctx = static_cast<IsrContext*>(arg);
    GpioEvent event;
    event.pin = ctx->pin;
    event.timestampUs = esp_timer_get_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(ctx->queue, &event, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void GpioInterruptManager::dispatchTask(void* arg) {
    auto* self = static_cast<GpioInterruptManager*>(arg);
    GpioEvent event;

    while (self->taskRunning) {
        if (xQueueReceive(self->eventQueue, &event, pdMS_TO_TICKS(DISPATCH_TIMEOUT_MS)) == pdTRUE) {
            auto pinIdx = static_cast<size_t>(event.pin);

            if (pinIdx < MAX_GPIO_PINS && self->registrations[pinIdx].registered && self->registrations[pinIdx].enabled &&
                self->registrations[pinIdx].callback) {
                self->registrations[pinIdx].callback(event.pin);
            }
        }
    }

    vTaskDelete(nullptr);
}

gpio_int_type_t GpioInterruptManager::toGpioIntrType(EdgeType edge) {
    switch (edge) {
        case EdgeType::Rising:
            return GPIO_INTR_POSEDGE;
        case EdgeType::Falling:
            return GPIO_INTR_NEGEDGE;
        case EdgeType::Both:
            return GPIO_INTR_ANYEDGE;
        default:
            return GPIO_INTR_DISABLE;
    }
}

bool GpioInterruptManager::isValidPin(gpio_num_t pin) const { return pin >= 0 && static_cast<size_t>(pin) < MAX_GPIO_PINS; }

}  // namespace GpioInterruptManager
}  // namespace WroRobotSoftware
