idf_component_register(
    SRCS
        "src/Encoder.cpp"
        "src/EncoderProcessing.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_pcnt
        esp_timer
        freertos
        log
        Interfaces
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)