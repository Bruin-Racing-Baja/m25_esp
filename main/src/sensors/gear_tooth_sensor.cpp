#include "gear_tooth_sensor.h"
#include "esp_cpu.h"
#include "../constants.h"

/* Update sensor count and time values. */
void GearToothSensor::update() {
    uint64_t cur_time_us = esp_cpu_get_cycle_count() * 1000000ULL / CPU_HZ;
    
    portENTER_CRITICAL_ISR(&mux);

    if (cur_time_us - last_time_us > min_time_diff_us) 
    {
        if (count % sample_window == 0) 
        {
            time_diff_us = cur_time_us - last_sample_time_us;
            last_sample_time_us = cur_time_us; 
        }
        count++; 
        last_time_us = cur_time_us;
    }

    portEXIT_CRITICAL_ISR(&mux); 
}

/* Calculate RPM from most recent gear count and time values. */
void GearToothSensor::calculate_rpm() {

    uint64_t local_time_diff_us; 
    portENTER_CRITICAL(&mux); 
    local_time_diff_us = time_diff_us; 
    portEXIT_CRITICAL(&mux); 

    if (local_time_diff_us != 0) {
        rpm = (float)sample_window / counts_per_rot / 
                local_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE; 
    }
}