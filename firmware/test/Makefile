# Host-side unit tests for the firmware motion path.
#
#   cd firmware/test && make && ./test_motion
#
# The real StepperController, HomingController, MotionPlanner and SerialProtocol
# are compiled against the Arduino mock in mock/, so these tests exercise the
# code that runs on the Teensy rather than a re-implementation of it.

CXX      ?= g++
CXXFLAGS ?= -std=c++14 -O2 -g -Wall -Wextra -Wno-unused-parameter -Imock -I..

SOURCES = test_motion.cpp \
          mock/Arduino.cpp \
          ../MotionPlanner.cpp \
          ../StepperController.cpp \
          ../HomingController.cpp \
          ../SafetyMonitor.cpp \
          ../SerialProtocol.cpp

TARGET = test_motion

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(SOURCES) $(wildcard ../*.h) $(wildcard mock/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES) -lm

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
