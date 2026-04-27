#include "BoardConfig.hpp"
#include "DeviceManager.hpp"
#include "esp_log.h"

using namespace WroRobotSoftware::DeviceManager;

static const char* TAG = "main";

extern "C" void app_main(void) {
    static DeviceManager deviceManager(deviceManagerConfig);

    esp_err_t ret = deviceManager.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DeviceManager init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = deviceManager.start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DeviceManager start failed: %s", esp_err_to_name(ret));
        return;
    }
}
