#pragma once

#include <stdint.h>

#include <atomic>

void callbackEx(int pin, int value, uint32_t tick, void* data);

class Encoder {
    friend void callbackEx(int pin, int value, uint32_t tick, void* data);

private:
    const int pin_a;
    const int pin_b;

    int pin_previous;

    int value_a;
    int value_b;

    std::atomic<int> count;

    void callback(int pin, int value, uint32_t tick);

public:
    Encoder(int pin_a, int pin_b);
    ~Encoder();

    int get_count();
    void set_count(int count);
};
