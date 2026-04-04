idf_component_register(
    SRCS "src/DRV8876.cpp"
         "src/GpioController.cpp"
         "src/FaultHandler.cpp"
         "src/PwmController.cpp"
         "src/MotionRamp.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio Interfaces
)