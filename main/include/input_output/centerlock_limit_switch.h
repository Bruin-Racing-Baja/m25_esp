#ifndef CENTERLOCK_LIMIT_SWITCH_H
#define CENTERLOCK_LIMIT_SWITCH_H

#include "gpio_wrapper.h"
#include "constants.h"
class CenterlockLimitSwitch {
public:
    CenterlockLimitSwitch() : engage_pressed(false), disengage_pressed(false) {}

    void isr_update_outbound(){
        int interrupt_pressed = digitalRead(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN);

        if (interrupt_pressed){
            disengage_pressed = true;
        } 
    }
    void isr_update_inbound(){
        int interrupt_pressed = digitalRead(CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN);

        if (interrupt_pressed){
            engage_pressed = true;
        } 
    }

    // engaged functions
    
    bool is_outbound_engaged(){
        return disengage_pressed;
    }

    bool is_inbound_engaged(){
        return engage_pressed;
    }

private:
    bool engage_pressed;
    bool disengage_pressed;
};

#endif // CENTERLOCK_LIMIT_SWITCH_H