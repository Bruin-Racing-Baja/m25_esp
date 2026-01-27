
idf_component_register(
    SRCS 
        "src/ecvt_controller.cpp" 
        "src/centerlock_controller.cpp" 
        "src/odrive.cpp"
        "src/gpio_wrapper.cpp"
        "src/sensors/brake_pot_sensor.cpp" 
        "src/sensors/gear_tooth_sensor.cpp"
        "src/sensors/throt_pot_sensor.cpp"
        "app_main.cpp"

    INCLUDE_DIRS 
        "include"

    REQUIRES
        esp_driver_twai
        driver 
        esp_adc 
        esp_timer
)


