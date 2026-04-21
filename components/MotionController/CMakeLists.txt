idf_component_register(
    SRCS "src/MotionController.cpp"
    INCLUDE_DIRS "include"
    REQUIRES DRV8876 Servo Interfaces log
)
