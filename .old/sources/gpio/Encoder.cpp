#include <pigpio.h>

#include "Encoder.hpp"
#include "Factory.hpp"

void callbackEx(int pin, int value, uint32_t tick, void* data)
{
    Encoder* rotary_encoder = reinterpret_cast<Encoder*>(data);
    rotary_encoder->callback(pin, value, tick);
}

void Encoder::callback(int pin, int value, uint32_t tick)
{
    if (pin == pin_a) {
        value_a = value;
    } else if (pin == pin_b) {
        value_b = value;
    }

    if (pin != pin_previous) {
        pin_previous = pin;

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

Encoder::Encoder(int pin_a, int pin_b)
    : pin_a(pin_a)
    , pin_b(pin_b)
    , pin_previous(0)
    , value_a(0)
    , value_b(0)
    , count(0)
{
    Factory* factory = &Factory::getFactory();

    gpioSetMode(pin_a, PI_INPUT);
    gpioSetMode(pin_b, PI_INPUT);

    gpioSetPullUpDown(pin_a, PI_PUD_UP);
    gpioSetPullUpDown(pin_b, PI_PUD_UP);

    gpioSetAlertFuncEx(pin_a, callbackEx, this);
    gpioSetAlertFuncEx(pin_b, callbackEx, this);
}

Encoder::~Encoder()
{
    gpioSetAlertFuncEx(pin_a, nullptr, this);
    gpioSetAlertFuncEx(pin_b, nullptr, this);
}

int Encoder::get_count()
{
    return count;
}

void Encoder::set_count(int new_count)
{
    count = new_count;
}
