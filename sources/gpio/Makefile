CXX := g++

PY_CPPFLAGS  := $(shell python3-config --cflags)
CXXFLAGS := $(PY_CPPFLAGS) -fpic

PY_LDFLAGS := $(shell python3-config --ldflags)
LDFLAGS := $(PY_LDFLAGS) -shared -lpigpio

SWIG := swig

SWIG_FLAGS := -c++ -python

OBJS = gpio_wrap.o Factory.o Motor.o Encoder.o

ALL = _gpio.so

all: $(ALL)

_gpio.so: $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) -c $< -o $@ $(CXXFLAGS)

gpio_wrap.o: gpio_wrap.cxx
	$(CXX) -c $< -o $@ $(CXXFLAGS)

gpio_wrap.cxx: gpio.i
	$(SWIG) $(SWIG_FLAGS) $<
