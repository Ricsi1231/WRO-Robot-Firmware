idf_component_register(
    SRCS "src/GpioInterruptManager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer freertos log Interfaces
)
