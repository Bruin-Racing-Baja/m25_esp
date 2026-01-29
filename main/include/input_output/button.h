#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button(int pin) : pin(pin), button_pressed(false), last_state(false){}

    bool button_pressed_isr(){
        return button_pressed && !last_state;
    }
    bool button_released_isr(){
        return !button_pressed && last_state;
    }

    bool read_button_state(){
        return button_pressed;
    }
    bool read_last_state(){
        return last_state;
    }

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H