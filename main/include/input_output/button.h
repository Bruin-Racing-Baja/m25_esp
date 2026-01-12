#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button() {}

    bool readPin();
    bool isPressed();
    bool readLastState();

    bool pressTriggered();
    bool releaseTriggered();

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H