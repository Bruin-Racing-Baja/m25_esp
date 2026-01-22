
idf_component_register(
    SRCS 
        "src/ecvt_controller.cpp" 
        "src/centerlock_controller.cpp" 
        "src/odrive.cpp"
        "src/gpio_wrapper.cpp"
        "app_main.cpp"

    INCLUDE_DIRS 
        "include" 
    REQUIRES
        driver esp_adc esp_timer
        
        
)


