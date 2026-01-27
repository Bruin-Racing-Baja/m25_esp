#ifndef LED_H
#define LED_H

#include <vector>
#include "gpio_wrapper.h"

using std::vector;

class LED {
public:
    LED (vector<int> pins) {
        this->pins = pins;
    }

    void set_flash_time(int time){
        flash_time = time;
    }

    void turn_on_all(){
        for (int pin : pins){
            digitalWrite(pin, HIGH);
        }
    }

    void turn_off_all(){
        for (int pin : pins){
            digitalWrite(pin, LOW);
        }
    }

    void turn_on_led(int led_pin){
        digitalWrite(led_pin, HIGH);
    }
    void turn_off_led(int led_pin){
        digitalWrite(led_pin, LOW);
    }

private:
    vector<int> pins;

    bool led_state;
    int flash_time;
};

#endif // LED_H