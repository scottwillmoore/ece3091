#pragma once

#include <stdint.h>

class Motor {
public:
    Motor(uint8_t direction_pin, uint8_t speed_pin);
    ~Motor();

    bool get_direction();
    void set_direction(bool direction);

    double get_speed();
    void set_speed(double speed);

private:
    const uint32_t pwm_frequency;

    const uint8_t direction_pin;
    const uint8_t speed_pin;

    bool is_hardware_pwm;

    bool direction;
    double speed;
};
