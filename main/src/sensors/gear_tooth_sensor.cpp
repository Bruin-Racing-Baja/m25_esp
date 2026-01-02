#include "gear_tooth_sensor.h"
#include "esp_cpu.h"
#include "../constants.h"

/* Update sensor count and time values. */
void GearToothSensor::update() {
    uint64_t cur_time_us = esp_cpu_get_cycle_count() * 1e6 / CPU_HZ;
    if (cur_time_us - last_time_us < min_time_diff_us) return;

    if (count % sample_window == 0) {
        time_diff_us = cur_time_us - last_sample_time_us;
        last_sample_time_us = cur_time_us; 
    }
    count++; 
    last_time_us = cur_time_us;
}

/* Calculate RPM from most recent gear count and time values. */
void GearToothSensor::calculate_rpm() {
    if (time_diff_us != 0) {
        rpm = (float)sample_window / counts_per_rot / 
                time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE; 
    }
}