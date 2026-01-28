#include "shift_register.h"

ShiftRegister::ShiftRegister(int _ser_in_pin, int _srck_pin, int _rck_pin)
    : ser_in_pin(_ser_in_pin), srck_pin(_srck_pin), rck_pin(_rck_pin), data(0) {
    pinMode(ser_in_pin, OUTPUT);
    pinMode(srck_pin, OUTPUT);
    pinMode(rck_pin, OUTPUT);
}

void ShiftRegister::write_byte(uint8_t byte)
{
    digitalWrite(latchPin, LOW);

    for (int i = 7; i >= 0; i--) {
        digitalWrite(dataPin, (byte >> i) & 0x01);
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
    }

    digitalWrite(latchPin, HIGH);

    data = byte;
}


void ShiftRegister::write_led(int led_num, bool value) {
    write_byte((data & ~(1u<<led_num)) | (value<<led_num)); //replaces n-th bit with value and writes to register
}