#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button() {}

    bool read_pin();
    bool is_pressed();
    bool read_last_state();

    bool press_triggered();
    bool release_triggered();

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H