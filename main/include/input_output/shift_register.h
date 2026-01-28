#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include "gpio_wrapper.h"

class ShiftRegister {
public:
    ShiftRegister(int _ser_in_pin, int _srck_pin, int _rck_pin);
    
    void write_byte(uint8_t byte); 
    void write_led(uint8_int led_num, bool value); 

    uint8_t get_data() const {return data;}
    
private:
    int ser_in_pin;   // serial input
    int srck_pin;     // shift register clock
    int rck_pin;      // register clock (activates latch)
    uint8_t data; //current data on shift register (default 0)
};

#endif // SHIFT_REGISTER_H