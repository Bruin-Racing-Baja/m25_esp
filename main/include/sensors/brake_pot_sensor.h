#ifndef BRAKE_POT_H
#define BRAKE_POT_H

#include "sensor.h"
#include "../macros.h"
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

class BrakePot: public Sensor {
public: 
    BrakePot(uint32_t min_brake_, uint32_t max_brake_): 
        raw_brake(0),
        brake(0.0f),
        min_brake(min_brake_),
        max_brake(max_brake_)
        {}

    bool init(); 
    virtual void update_isr();
    
    inline uint32_t get_raw_brake() { return raw_brake; }
    inline float get_brake() { return map_int_to_float(raw_brake, min_brake, max_brake, 0, 1); }

private:
    const uint32_t min_brake; 
    const uint32_t max_brake; 

    uint32_t raw_brake; 
    float brake; 
};

#endif // BRAKE_POT_H