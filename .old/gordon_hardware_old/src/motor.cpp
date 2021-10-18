#include <pigpiod_if2.h>
#include <stdint.h>

#include <atomic>
#include <stdexcept>

#include "gordon_hardware/motor.hpp"

Motor::Motor(uint8_t direction_pin, uint8_t speed_pin)
    : pwm_frequency(10000)
    , direction_pin(direction_pin)
    , speed_pin(speed_pin)
    , direction(true)
    , speed(0)
{
    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        throw std::runtime_error("Failed to initialize pigpio");
    }

    set_mode(pi, direction_pin, PI_OUTPUT);
    set_mode(pi, speed_pin, PI_OUTPUT);

    set_pull_up_down(pi, direction_pin, PI_PUD_DOWN);
    set_pull_up_down(pi, speed_pin, PI_PUD_DOWN);

    gpio_write(pi, direction_pin, 0);

    if (hardware_PWM(pi, speed_pin, pwm_frequency, 0) == PI_NOT_HPWM_GPIO) {
        throw std::runtime_error("Failure to initialize hardware PWM");
    }
}

Motor::~Motor()
{
    gpio_write(pi, direction_pin, 0);
    gpio_write(pi, speed_pin, 0);
}
