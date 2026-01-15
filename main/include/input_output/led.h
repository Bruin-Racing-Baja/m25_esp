#ifndef LED_H
#define LED_H

class LED {
public:
    LED() {}
    void write_pin(bool state);
    void pin_mode(int mode);
    void set_flash_time(int time);

private:
    int pin;
    bool led_state;
    int flash_time;
};

#endif // LED_H