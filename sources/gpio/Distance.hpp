#pragma once

#include <stdint.h>

#include <atomic>

void callbackEx(int pin, int value, uint32_t tick, void* data);

class Distance {
    friend void callbackEx(int pin, int value, uint32_t tick, void* data);

private:
    const int trigger_pin;
    const int echo_pin;

    void callback(int pin, int value, uint32_t tick);

public:
    Distance(int trigger_pin, int echo_pin);
    ~Distance();
};
