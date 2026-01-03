#ifndef ECVT_LIMIT_SWITCH_H
#define ECVT_LIMIT_SWITCH_H

class ECVTLimitSwitch {
public:
    ECVTLimitSwitch() {}

    // Returns true if the limit switch is activated
    bool inboundTriggered() const {
        return inbound_limit_switch_activated;
    }
    bool outboundTriggered() const {
        return outbound_limit_switch_activated;
    }
    bool engageTriggered() const {
        return engage_limit_switch_activated;
    }

    // Returns true if the switch was pressed
    bool inboundPressed() const {
        return inbound_pressed;
    }
    bool outboundPressed() const {
        return outbound_pressed;
    }
    bool engagePressed() const {
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