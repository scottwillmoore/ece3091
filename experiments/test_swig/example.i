%module example

%{
#define SWIG_FILE_WITH_INIT
#include "example.hpp"
%}

%include "example.hpp"
