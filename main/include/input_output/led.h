#ifndef LED_H
#define LED_H

#include <vector>
using std::vector;

class LED {
public:
    LED (vector<int> pins) {
        pins = pins;
    }
    void write_pin(bool state);
    void pin_mode(int mode);
    void set_flash_time(int time);

    void turn_on_all();
    void turn_off_all();

    void turn_on_led(int led_pin);
    void turn_off_led(int led_pin);

private:
    vector<int> pins;

    bool led_state;
    int flash_time;
};

#endif // LED_H