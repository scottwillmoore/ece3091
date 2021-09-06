#include <pigpio.h>
#include <unistd.h>

#include <iostream>

#include "Motor.hpp"
#include "RotaryEncoder.hpp"

int main(void)
{
    if (gpioInitialise() < 0) {
        return 1;
    }

    {
        Motor left_motor(6, 13);
        RotaryEncoder left_encoder(9, 11);

        left_motor.setSpeed(1.0);

        sleep(5);

        std::cout << left_encoder.getCount() << std::endl;

        const double MOTOR_TO_GEAR_RATIO = 1.0 / (32.0 * 344.2);
        std::cout << left_encoder.getCount() * MOTOR_TO_GEAR_RATIO << std::endl;
    }

    gpioTerminate();
}
