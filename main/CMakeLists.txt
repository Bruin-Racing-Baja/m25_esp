idf_component_register(
    SRCS 
        "src/ecvt_controller.cpp" 
        "src/centerlock_controller.cpp" 
        "src/odrive.cpp"
        "src/constants.cpp" 
        "src/sensors/brake_pot_sensor.cpp" 
        "src/sensors/gear_tooth_sensor.cpp"
        "app_main.cpp"
    INCLUDE_DIRS 
        "include" 
)


