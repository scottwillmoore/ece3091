swig -c++ -python device.i
g++ -shared -lpigpio -o _device.so GPIO.cpp Motor.cpp RotaryEncoder.cpp device_wrap.cxx
