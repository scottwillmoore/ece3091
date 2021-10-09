#include <pigpio.h>

#include <algorithm>
#include <cmath>

#include "Factory.hpp"
#include "Motor.hpp"

Motor::Motor(int pin_direction, int pin_speed)
    : pin_direction(pin_direction)
    , pin_speed(pin_speed)
    , pwm_frequency(10000)
    , is_hardware_pwm(true)
    , direction(true)
    , speed(0.0)
{
    Factory* factory = &Factory::getFactory();

    gpioSetMode(pin_direction, PI_OUTPUT);
    gpioSetMode(pin_speed, PI_OUTPUT);

    gpioSetPullUpDown(pin_direction, PI_PUD_DOWN);
    gpioSetPullUpDown(pin_speed, PI_PUD_DOWN);

    gpioWrite(pin_direction, 0);

    if (gpioHardwarePWM(pin_speed, pwm_frequency, 0) == PI_NOT_HPWM_GPIO) {
        is_hardware_pwm = false;

        gpioSetPWMfrequency(pin_speed, pwm_frequency);
        gpioPWM(pin_speed, 0);
    }
}

Motor::~Motor()
{
    gpioWrite(pin_direction, 0);

    if (is_hardware_pwm) {
        gpioHardwarePWM(pin_speed, pwm_frequency, 0);
    } else {
        gpioPWM(pin_speed, 0);
    }
}

bool Motor::get_direction()
{
    return direction;
}

void Motor::set_direction(bool new_direction)
{
    direction = new_direction;

    if (direction) {
        gpioWrite(pin_direction, 0);
    } else {
        gpioWrite(pin_direction, 1);
    }
}

double Motor::get_speed()
{
    return speed;
}

void Motor::set_speed(double new_speed)
{
    speed = new_speed;

    if (is_hardware_pwm) {
        const int min = 0;
        const int max = 1 * 1000 * 1000;
        unsigned duty_cycle = std::max(min, std::min(max, (int)std::floor(speed * max)));

        gpioHardwarePWM(pin_speed, pwm_frequency, duty_cycle);
    } else {
        const int min = 0;
        const int max = 256;
        unsigned duty_cycle = std::max(min, std::min(max, (int)std::floor(speed * max)));

        gpioPWM(pin_speed, duty_cycle);
    }
}

void Motor::backward()
{
    backward(1.0);
}

void Motor::backward(double speed)
{
    set_direction(false);
    set_speed(speed);
}

void Motor::forward()
{
    forward(1.0);
}

void Motor::forward(double speed)
{
    set_direction(true);
    set_speed(speed);
}

void Motor::reverse()
{
    set_direction(!direction);
}

void Motor::stop()
{
    set_speed(0.0);
}
