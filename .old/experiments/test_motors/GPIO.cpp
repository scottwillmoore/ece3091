#include "GPIO.hpp"

void set_up()
{
    gpioInitialise();
}

void tear_down()
{
    gpioTerminate();
}
