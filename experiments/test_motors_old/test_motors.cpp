#include <iostream>
#include <stdexcept>

#include <pigpio.h>

#include "test_motors.hpp"

using namespace std;

// Board
// Pin
// InputPin
// OutputPin
// InputPin <- DigitalInput
// OutputPin <- DigitalOutput
// PWMPin <- PWMOutput

bool GPIOContext::isInitialised = false;

GPIOContext::GPIOContext()
{
    if (isInitialised)
        throw runtime_error("The GPIO context has already initialised");

    if (gpioInitialise() < 0)
        throw runtime_error("The GPIO context could not be initialised");

    isInitialised = true;
}

GPIOContext::~GPIOContext()
{
    if (isInitialised)
    {
        gpioTerminate();
        isInitialised = false;
    }
}

int main()
{
    auto gpioContext = GPIOContext();

    std::cout << "Hello, world!" << std::endl;

    return 0;
}
