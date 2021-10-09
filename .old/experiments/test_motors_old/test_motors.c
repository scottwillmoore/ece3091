#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>

#include "rotary_encoder.h"

#define LEFT_MOTOR_ENABLE_PIN 13
#define LEFT_MOTOR_PHASE_PIN 6

#define LEFT_ENCODER_A_PIN 9
#define LEFT_ENCODER_B_PIN 11

// gcc -lpigpio -lrt -o test_motors test_motors.c rotary_encoder.c

static int steps = 0;

void callback(int direction)
{
    steps -= direction;
}

int main(int argc, char *argv[])
{
    Pi_Renc_t *renc;

    if (gpioInitialise() < 0)
        return 1;

    renc = Pi_Renc(9, 11, callback);

    printf("steps = %d\n", steps);

    gpioPWM(LEFT_MOTOR_ENABLE_PIN, 255);

    sleep(5);

    gpioPWM(LEFT_MOTOR_ENABLE_PIN, 0);

    // Pi_Renc_cancel(renc);

    printf("steps = %d\n", steps);

    double ratio = 0.00009079;

    printf("turns = %f\n", (double)steps * ratio);

    gpioTerminate();
}
