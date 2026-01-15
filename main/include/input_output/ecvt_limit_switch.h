#ifndef ECVT_LIMIT_SWITCH_H
#define ECVT_LIMIT_SWITCH_H

class ECVTLimitSwitch {
public:
    ECVTLimitSwitch() {}

    // Returns true if the limit switch is activated
    bool inbound_triggered() const {
        return inbound_limit_switch_activated;
    }
    bool outbound_triggered() const {
        return outbound_limit_switch_activated;
    }
    bool engage_triggered() const {
        return engage_limit_switch_activated;
    }

    // Returns true if the switch was pressed
    bool inbound_pressed() const {
        return inbound_pressed;
    }
    bool outbound_pressed() const {
        return outbound_pressed;
    }
    bool engage_pressed() const {
        return engage_pressed;
    }
    
    void begin();
    void update();
    void end();

private:
    bool inbound_limit_switch_activated;
    bool outbound_limit_switch_activated;
    bool engage_limit_switch_activated;

    bool inbound_pressed;
    bool outbound_pressed;
    bool engage_pressed;
};

#endif // ECVT_LIMIT_SWITCH_H