#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button() {}

    bool press_triggered(){
        return button_pressed && !last_state;
    }
    bool release_triggered(){
        return !button_pressed && last_state;
    }

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H