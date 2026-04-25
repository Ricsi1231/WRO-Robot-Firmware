idf_component_register(
    SRCS "src/ReflectanceSensor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_adc freertos log Interfaces
)
