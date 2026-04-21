idf_component_register(
    SRCS "src/ServoChannel.cpp"
         "src/ServoRamp.cpp"
         "src/ServoManager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio freertos Interfaces log
)
