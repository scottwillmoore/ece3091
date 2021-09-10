%include <attribute.i>

%{
#include "Motor.hpp"
%}

%attribute(Motor, bool, direction, get_direction, set_direction)
%attribute(Motor, double, speed, get_speed, set_speed)

%include "Motor.hpp"
