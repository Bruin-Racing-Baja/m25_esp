#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button() {}

    bool press_triggered();
    bool release_triggered();

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H