idf_component_register(
    SRCS "src/DeviceManager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        MotionController
        DRV8876
        Servo
        Encoder
        ReflectanceSensor
        GpioInterruptManager
        PID
        PathPlanning
        Interfaces
        freertos
        log
)
