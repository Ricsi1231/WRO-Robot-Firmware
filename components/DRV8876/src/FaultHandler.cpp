#include "FaultHandler.hpp"

namespace WroRobotSoftware {
namespace DRV8876 {

FaultHandler::FaultHandler(FaultHandler&& other) noexcept
    : nFaultPin(other.nFaultPin),
      faultTriggered(other.faultTriggered.load(std::memory_order_relaxed)),
      faultCallback(std::move(other.faultCallback)),
      initialized(other.initialized) {
    other.nFaultPin = GPIO_NUM_NC;
    other.faultTriggered.store(false, std::memory_order_relaxed);
    other.faultCallback = nullptr;
    other.initialized = false;
}

FaultHandler& FaultHandler::operator=(FaultHandler&& other) noexcept {
    if (this != &other) {
        nFaultPin = other.nFaultPin;
        faultTriggered.store(other.faultTriggered.load(std::memory_order_relaxed), std::memory_order_relaxed);
        faultCallback = std::move(other.faultCallback);
        initialized = other.initialized;
        other.nFaultPin = GPIO_NUM_NC;
        other.faultTriggered.store(false, std::memory_order_relaxed);
        other.faultCallback = nullptr;
        other.initialized = false;
    }
    return *this;
}

esp_err_t FaultHandler::init(gpio_num_t nFaultPin) {
    this->nFaultPin = nFaultPin;

    gpio_config_t faultConf = {};
    faultConf.intr_type = GPIO_INTR_NEGEDGE;
    faultConf.mode = GPIO_MODE_INPUT;
    faultConf.pin_bit_mask = (1ULL << nFaultPin);
    faultConf.pull_up_en = GPIO_PULLUP_ENABLE;
    esp_err_t ret = gpio_config(&faultConf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(nFAULT) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR service already installed");
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(nFaultPin, faultISR, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "nFAULT ISR attached on GPIO %d", nFaultPin);
    return ESP_OK;
}

bool FaultHandler::isFaultTriggered() const { return faultTriggered.load(std::memory_order_relaxed); }

void FaultHandler::clearFaultFlag() { faultTriggered.store(false, std::memory_order_relaxed); }

bool FaultHandler::getAndClearFault() { return faultTriggered.exchange(false, std::memory_order_acq_rel); }

void FaultHandler::setFaultCallback(const std::function<void()>& cb) { faultCallback = cb; }

void FaultHandler::processFaultEvent() {
    bool hadFault = getAndClearFault();
    if (hadFault == false) {
        return;
    }

    if (faultCallback) {
        faultCallback();
    }
}

void IRAM_ATTR FaultHandler::faultISR(void* arg) {
    auto* self = static_cast<FaultHandler*>(arg);
    self->faultTriggered.store(true, std::memory_order_relaxed);
}

}  // namespace DRV8876
}  // namespace WroRobotSoftware
