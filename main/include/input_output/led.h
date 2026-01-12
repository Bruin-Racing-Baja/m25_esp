#ifndef LED_H
#define LED_H

class LED {
public:
    LED() {}
    void writePin(bool state);
    void pinMode(int mode);
    void setFlashTime(int time);

private:
    int pin;
    bool led_state;
    int flash_time;
};

#endif // LED_H