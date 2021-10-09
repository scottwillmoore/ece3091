#include <pigpio.h>

#include <iostream>
#include <stdexcept>

#include "Factory.hpp"

Factory& Factory::getFactory()
{
    static Factory instance;
    return instance;
}

Factory::Factory()
{
    int result = gpioInitialise();
    if (result < 0) {
        throw std::runtime_error("Failed to initialise pigpio");
    }
}

Factory::~Factory()
{
    gpioTerminate();
}
