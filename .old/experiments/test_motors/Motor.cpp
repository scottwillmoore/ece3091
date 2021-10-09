#include <algorithm>
#include <cmath>

#include "Motor.hpp"

Motor::Motor(int pin_direction, int pin_enable)
    : Motor(pin_direction, pin_enable, DEFAULT_PWM_FREQUENCY)
{
}

Motor::Motor(int pin_direction, int pin_enable, unsigned pwm_frequency)
    : pin_direction(pin_direction)
    , pin_enable(pin_enable)
    , pwm_frequency(pwm_frequency)
    , is_hardware_pwm(true)
    , direction(true)
    , speed(0.0)
{
    gpioSetMode(pin_direction, PI_OUTPUT);
    gpioSetMode(pin_enable, PI_OUTPUT);

    gpioSetPullUpDown(pin_direction, PI_PUD_OFF);
    gpioSetPullUpDown(pin_enable, PI_PUD_OFF);

    gpioWrite(pin_direction, 0);

    if (gpioHardwarePWM(pin_enable, pwm_frequency, 0) == PI_NOT_HPWM_GPIO) {
        is_hardware_pwm = false;

        gpioSetPWMfrequency(pin_enable, pwm_frequency);
        gpioPWM(pin_enable, 0);
    }
}

Motor::~Motor()
{
    if (is_hardware_pwm) {
        gpioHardwarePWM(pin_enable, pwm_frequency, 0);
    } else {
        gpioPWM(pin_enable, 0);
    }
    gpioWrite(pin_direction, 0);
}

bool Motor::getDirection()
{
    return direction;
}

void Motor::setDirection(bool new_direction)
{
    direction = new_direction;

    if (direction) {
        gpioWrite(pin_direction, 1);
    } else {
        gpioWrite(pin_direction, 0);
    }
}

double Motor::getSpeed()
{
    return speed;
}

void Motor::setSpeed(double new_speed)
{
    speed = new_speed;

    if (is_hardware_pwm) {
        const int min = 0;
        const int max = 1 * 1000 * 1000;
        unsigned duty_cycle = std::max(min, std::min(max, (int)std::floor(speed * max)));

        gpioHardwarePWM(pin_enable, pwm_frequency, duty_cycle);
    } else {
        const int min = 0;
        const int max = 256;
        unsigned duty_cycle = std::max(min, std::min(max, (int)std::floor(speed * max)));

        gpioPWM(pin_enable, duty_cycle);
    }
}
