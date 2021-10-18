#pragma once

#include <stdint.h>

#include <atomic>

class RotaryEncoder {
public:
    RotaryEncoder(uint8_t a_pin, uint8_t b_pin);
    ~RotaryEncoder();

    const uint8_t a_pin;
    const uint8_t b_pin;

    std::atomic<int32_t> count;

private:
    static void callbackEx(int pi, unsigned pin, unsigned value, uint32_t tick, void* data);
    void callback(int pi, unsigned pin, unsigned value, uint32_t tick);

    int pi;

    int a_callback;
    uint8_t a_value;

    int b_callback;
    uint8_t b_value;

    uint8_t previous_pin;
};
