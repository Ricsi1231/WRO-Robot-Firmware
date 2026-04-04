#include "GpioController.hpp"

namespace WroRobotSoftware {
namespace DRV8876 {

GpioController::GpioController(GpioController&& other) noexcept : phPin(other.phPin), nSleepPin(other.nSleepPin), initialized(other.initialized) {
    other.phPin = GPIO_NUM_NC;
    other.nSleepPin = GPIO_NUM_NC;
    other.initialized = false;
}

GpioController& GpioController::operator=(GpioController&& other) noexcept {
    if (this != &other) {
        phPin = other.phPin;
        nSleepPin = other.nSleepPin;
        initialized = other.initialized;
        other.phPin = GPIO_NUM_NC;
        other.nSleepPin = GPIO_NUM_NC;
        other.initialized = false;
    }
    return *this;
}

esp_err_t GpioController::init(gpio_num_t phPin, gpio_num_t nSleepPin) {
    this->phPin = phPin;
    this->nSleepPin = nSleepPin;

    gpio_config_t ioConf = {};
    ioConf.intr_type = GPIO_INTR_DISABLE;
    ioConf.mode = GPIO_MODE_OUTPUT;
    ioConf.pin_bit_mask = (1ULL << phPin);
    esp_err_t ret = gpio_config(&ioConf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(PH) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (nSleepPin != GPIO_NUM_NC) {
        ioConf.pin_bit_mask = (1ULL << nSleepPin);
        ret = gpio_config(&ioConf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config(nSLEEP) failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    initialized = true;
    return ESP_OK;
}

void GpioController::setDirection(bool level) {
    if (initialized == false) {
        ESP_LOGE(TAG, "GpioController is not initialized");
        return;
    }
    gpio_set_level(phPin, level);
}

void GpioController::setSleep(bool awake) {
    if (initialized == false) {
        ESP_LOGE(TAG, "GpioController is not initialized");
        return;
    }
    if (nSleepPin == GPIO_NUM_NC) {
        return;
    }
    gpio_set_level(nSleepPin, awake);
}

void GpioController::setLevel(gpio_num_t pin, bool level) { gpio_set_level(pin, level); }

}  // namespace DRV8876
}  // namespace WroRobotSoftware
