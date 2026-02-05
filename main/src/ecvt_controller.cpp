#include "ecvt_controller.h"

#include <sensors/gear_tooth_sensor.h>
#include <input_output/shift_register.h>
#include <odrive.h> 
#include <constants.h>

#include "esp_timer.h"

ECVTController* ECVTController::instance = nullptr;

ECVTController::ECVTController(ShiftRegister* sr, bool wait_for_can)
    : primary_gts(ENGINE_GEARTOOTH_SENSOR_PIN, ENGINE_SAMPLE_WINDOW, ENGINE_COUNTS_PER_ROT), 
      secondary_gts(GEARBOX_GEARTOOTH_SENSOR_PIN, GEAR_SAMPLE_WINDOW, GEAR_COUNTS_PER_ROT),
      shift_reg(sr)
{
    instance = this; 
    init(wait_for_can);
}

void ECVTController::init(bool wait_for_can) 
{
    /* Set Pin Modes */
    pinMode(ECVT_LIMIT_SWITCH_INBOUND_PIN, PinMode::INPUT_ONLY);
    pinMode(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, PinMode::INPUT_ONLY);

    /* Initialize ODrive */
    odrive.init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    odrive.start();
    odrive.clear_errors(0);

    /* Flash While Wait for CAN Heartbeat */
    if (wait_for_can) {
        uint32_t led_flash_time_ms = 100;
        while (odrive.get_time_since_last_heartbeat() > 1e6) {
            shift_reg->write_all_leds(esp_timer_get_time() % (led_flash_time_ms * 2) < led_flash_time_ms);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    shift_reg->write_all_leds(false);

    bool homed = home_actuator(); 
    if (homed) {
        /* Add log statement if homed */
    }

    /* Attach Interrupts */
    attachInterrupt(ENGINE_GEARTOOTH_SENSOR_PIN, primary_isr, InterruptMode::RISING_EDGE);    
    attachInterrupt(GEARBOX_GEARTOOTH_SENSOR_PIN, secondary_isr, InterruptMode::RISING_EDGE);

    attachInterrupt(ECVT_LIMIT_SWITCH_INBOUND_PIN, inbound_isr, InterruptMode::RISING_EDGE);
    attachInterrupt(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, outbound_isr, InterruptMode::RISING_EDGE);
}

bool ECVTController::home_actuator() {
    /* Implement Actuator Homing */
    return true; 
}

void ECVTController::control_function() {
    /* Implement Controller */
} 

void IRAM_ATTR ECVTController::primary_isr(void* p) {
    if (instance) {
        instance->primary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::secondary_isr(void* p) {
    if (instance) {
        instance->primary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::outbound_isr(void* p) {
    if (instance) {
        /* set velocity to 0, hard stop */
    }
}

void IRAM_ATTR ECVTController::inbound_isr(void* p) {
    if (instance) {
        /* Set velocity to 0, hard stop */
    }
}