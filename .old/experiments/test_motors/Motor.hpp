#pragma once

#include <pigpio.h>

const unsigned DEFAULT_PWM_FREQUENCY = 100000;

class Motor {
private:
    const int pin_direction;
    const int pin_enable;

    const unsigned pwm_frequency;

    bool is_hardware_pwm;

    bool direction;
    double speed;

public:
    Motor(int pin_direction, int pin_enable);
    Motor(int pin_direction, int pin_enable, unsigned pwm_frequency);

    ~Motor();

    bool getDirection();
    void setDirection(bool direction);

    double getSpeed();
    void setSpeed(double speed);
};
