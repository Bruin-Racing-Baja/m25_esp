#ifndef CENTERLOCK_LIMIT_SWITCH_H
#define CENTERLOCK_LIMIT_SWITCH_H

class CenterlockLimitSwitch {
public:
    CenterlockLimitSwitch() {}

    void begin();
    void update();
    void end();

private:
    bool centerlock_limit_switch_activated;

    bool engage_pressed;
    bool disengage_pressed;
};

#endif // CENTERLOCK_LIMIT_SWITCH_H