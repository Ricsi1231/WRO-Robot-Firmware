idf_component_register(
    SRCS "src/PID.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer Interfaces
)