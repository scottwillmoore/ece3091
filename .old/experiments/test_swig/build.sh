swig -c++ -python example.i
g++ -shared -I/usr/include/python3.7 -o _example.o example.cpp example_wrap.cxx
