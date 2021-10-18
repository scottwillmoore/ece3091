#pragma once

#include <stdint.h>

class Motor {
public:
    Motor(uint8_t direction_pin, uint8_t speed_pin);
    ~Motor();

    const uint32_t pwm_frequency;

    const uint8_t direction_pin;
    const uint8_t speed_pin;

private:
    int pi;

    bool direction;
    double speed;
};
