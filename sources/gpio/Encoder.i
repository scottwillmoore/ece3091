%include <attribute.i>

%{
#include "Encoder.hpp"
%}

%attribute(Encoder, int, count, get_count, set_count)

%include "Encoder.hpp"
