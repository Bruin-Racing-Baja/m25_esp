#ifndef ECVT_LIMIT_SWITCH_H
#define ECVT_LIMIT_SWITCH_H

class ECVTLimitSwitch {
public:
    ECVTLimitSwitch() {}

    void begin();

    void isr_update_inbound();
    void isr_update_outbound();
    void isr_update_engage();

    void end();

    //triggered functions
    bool is_inbound_triggered() const {
        return inbound_limit_switch_activated; 
    }
    bool is_outbound_triggered() const {
        return outbound_limit_switch_activated;
    }
    bool is_engage_triggered() const {
        return engage_limit_switch_activated;
    }

    // engaged functions
    bool is_inbound_engaged() const {
        return inbound_engaged;
    }
    bool is_outbound_engaged() const {
        return outbound_engaged;
    }
    bool is_engage_engaged() const {
        return engage_engaged;
    }

private:
    bool inbound_limit_switch_activated;
    bool outbound_limit_switch_activated;
    bool engage_limit_switch_activated;

    bool inbound_engaged;
    bool outbound_engaged;
    bool engage_engaged;
};

#endif // ECVT_LIMIT_SWITCH_H