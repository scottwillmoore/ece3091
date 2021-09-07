#pragma once

#include <pigpio.h>
#include <stdint.h>

#include <atomic>

void callbackEx(int pin, int value, uint32_t tick, void* callback);

class RotaryEncoder {
    friend void callbackEx(int pin, int value, uint32_t tick, void* callback);

private:
    const int pin_a;
    const int pin_b;

    int last_pin;

    int value_a;
    int value_b;

    std::atomic_int count;

    void callback(int pin, int value, uint32_t tick);

public:
    RotaryEncoder(int pin_a, int pin_b);
    ~RotaryEncoder();

    int getCount();
    void setCount(int count);
};
