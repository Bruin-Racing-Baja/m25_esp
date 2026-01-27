#ifndef CENTERLOCK_LIMIT_SWITCH_H
#define CENTERLOCK_LIMIT_SWITCH_H

class CenterlockLimitSwitch {
public:
    CenterlockLimitSwitch() {}

    void begin();
    void isr_update_outbound();
    void isr_update_inbound();
    void end();

    // engaged functions
    
    bool is_outbound_engaged(){
        return disengage_pressed;
    }

    bool is_inbound_engaged(){
        return engage_pressed;
    }

private:
    bool centerlock_limit_switch_activated;

    bool engage_pressed;
    bool disengage_pressed;
};

#endif // CENTERLOCK_LIMIT_SWITCH_H