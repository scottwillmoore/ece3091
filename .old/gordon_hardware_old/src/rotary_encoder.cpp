#include <pigpiod_if2.h>
#include <stdint.h>

#include <atomic>
#include <stdexcept>

#include "gordon_hardware/rotary_encoder.hpp"

RotaryEncoder::RotaryEncoder(uint8_t a_pin, uint8_t b_pin)
    : count(0)
    , a_pin(a_pin)
    , a_value(0)
    , b_pin(b_pin)
    , b_value(0)
    , previous_pin(0)
{
    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        throw std::runtime_error("Failed to initialize pigpio");
    }

    set_mode(pi, a_pin, PI_INPUT);
    set_mode(pi, b_pin, PI_INPUT);

    set_pull_up_down(pi, a_pin, PI_PUD_UP);
    set_pull_up_down(pi, b_pin, PI_PUD_UP);

    a_callback = callback_ex(pi, a_pin, EITHER_EDGE, callbackEx, this);
    b_callback = callback_ex(pi, b_pin, EITHER_EDGE, callbackEx, this);
}

RotaryEncoder::~RotaryEncoder()
{
    callback_cancel(a_callback);
    callback_cancel(b_callback);

    pigpio_stop(pi);
}

void RotaryEncoder::callbackEx(int pi, unsigned pin, unsigned value, uint32_t tick, void* data)
{
    RotaryEncoder* rotary_encoder = reinterpret_cast<RotaryEncoder*>(data);
    rotary_encoder->callback(pi, pin, value, tick);
}

void RotaryEncoder::callback([[gnu::unused]] int pi, unsigned pin, unsigned value, [[gnu::unused]] uint32_t tick)
{
    if (pin == a_pin) {
        a_value = value;
    } else if (pin == b_pin) {
        b_value = value;
    }

    if (pin != previous_pin) {
        previous_pin = pin;

        if (pin == a_pin && value == 1) {
            if (b_value == 1) {
                count++;
            }
        } else if (pin == b_pin && value == 1) {
            if (a_value == 1) {
                count--;
            }
        }
    }
}
