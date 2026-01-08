#ifndef THROT_POT_H
#define THROT_POT_H

#include "sensors/sensor.h"
#include "macros.h"
#include <stdint.h>

class ThrotPot: public Sensor {
public: 
    ThrotPot(uint32_t min_throttle_, uint32_t max_throttle_):
        min_throttle(min_throttle_),
        max_throttle(max_throttle_),
        raw_throttle(0),
        throttle(0.0f)
        {}

    bool init(); 
    virtual void update_isr();
    
    inline uint32_t get_raw_throttle() { return raw_throttle; }
    inline float get_throttle() { return map_int_to_float(raw_throttle, min_throttle, max_throttle, 0, 1); }

private:
    const uint32_t min_throttle; 
    const uint32_t max_throttle; 

    uint32_t raw_throttle; 
    float throttle; 
};

#endif // THROT_POT_H