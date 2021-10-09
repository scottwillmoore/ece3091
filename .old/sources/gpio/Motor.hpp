#pragma once

class Motor {
private:
    const int pin_direction;
    const int pin_speed;

    const unsigned pwm_frequency;

    bool is_hardware_pwm;

    bool direction;
    double speed;

public:
    Motor(int pin_direction, int pin_speed);
    ~Motor();

    bool get_direction();
    void set_direction(bool direction);

    double get_speed();
    void set_speed(double speed);

    void backward();
    void backward(double speed);

    void forward();
    void forward(double speed);

    void reverse();

    void stop();
};
