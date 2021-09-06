#include "RotaryEncoder.hpp"

void callbackEx(int pin, int value, uint32_t tick, void* data)
{
    RotaryEncoder* rotary_encoder = reinterpret_cast<RotaryEncoder*>(data);
    rotary_encoder->callback(pin, value, tick);
}

void RotaryEncoder::callback(int pin, int value, uint32_t tick)
{
    if (pin == pin_a) {
        value_a = value;
    } else if (pin == pin_b) {
        value_b = value;
    }

    if (pin != last_pin) {
        last_pin = pin;

        if (pin == pin_a && value == 1) {
            if (value_b == 1) {
                count++;
            }
        } else if (pin == pin_b && value == 1) {
            if (value_a == 1) {
                count--;
            }
        }
    }
}

RotaryEncoder::RotaryEncoder(int pin_a, int pin_b)
    : pin_a(pin_a)
    , pin_b(pin_b)
    , last_pin(0)
    , value_a(0)
    , value_b(0)
    , count(0)
{
    gpioSetMode(pin_a, PI_INPUT);
    gpioSetMode(pin_b, PI_INPUT);

    gpioSetPullUpDown(pin_a, PI_PUD_UP);
    gpioSetPullUpDown(pin_b, PI_PUD_UP);

    gpioSetAlertFuncEx(pin_a, callbackEx, this);
    gpioSetAlertFuncEx(pin_b, callbackEx, this);
}

RotaryEncoder::~RotaryEncoder()
{
    gpioSetAlertFuncEx(pin_a, nullptr, this);
    gpioSetAlertFuncEx(pin_b, nullptr, this);
}

int RotaryEncoder::getCount()
{
    return int(count);
}

void RotaryEncoder::setCount(int new_count)
{
    count = new_count;
}
