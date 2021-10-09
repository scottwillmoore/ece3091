#include <pigpio.h>

#include "Distance.hpp"
#include "Factory.hpp"

void callbackEx(int pin, int value, uint32_t tick, void* data)
{
    Distance* distance_sensor = reinterpret_cast<Distance*>(data);
    distance_sensor->callback(pin, value, tick);
}

void Distance::callback(int pin, int value, uint32_t tick)
{
}

Distance::Distance(int trigger_pin, int echo_pin)
    : trigger_pin(trigger_pin)
    , echo_pin(echo_pin)
{
    Factory* factory = &Factory::getFactory();

    gpioSetMode(trigger_pin, PI_OUTPUT);

    gpioSetMode(echo_pin, PI_INPUT);
    gpioSetPullUpDown(echo_pin, PI_PUD_DOWN);
    gpioSetAlertFuncEx(echo_pin, callbackEx, this);
}

Distance::~Distance()
{
    gpioSetAlertFuncEx(echo_pin, nullptr, this);
}
