#pragma once

#include <cstdint>
#include "driver/gpio.h"

typedef void (*IsrHandler)(void*); // Function pointer type

// Constants for convenience
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;

enum class PinMode {
    INPUT_ONLY,
    OUTPUT_ONLY,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    INPUT_OUTPUT_OD // Input+Output
};

enum class InterruptMode {
    RISING_EDGE,
    FALLING_EDGE,
    ANY_CHANGE,
    LOW_LEVEL,
    HIGH_LEVEL
};

/**
 * @brief Configure a GPIO pin
 * @param pin GPIO number (e.g., 2, 4, GPIO_NUM_5)
 * @param mode The IO mode
 */
void pinMode(int pin, PinMode mode);


void digitalWrite(int pin, int level);
void digitalWrite(int pin, bool level);

int digitalRead(int pin);

int analogRead(int pin);
void analogWrite(int pin, int duty);

void attachInterrupt(int pin, IsrHandler handler, InterruptMode mode, void* arg = nullptr);