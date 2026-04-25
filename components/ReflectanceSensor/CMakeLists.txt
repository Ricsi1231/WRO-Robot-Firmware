idf_component_register(
    SRCS "src/ReflectanceSensor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc esp_timer freertos log Interfaces
)
