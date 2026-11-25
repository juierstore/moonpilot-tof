# Host side unit test for the pure C++ NLink parser.
#
#   make test     build and run
#   make clean
#
# The parser has no PX4 dependency, so any C++17 compiler works.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wshadow -Wpedantic
PARSER   := ../src/drivers/distance_sensor/tof_sense_ms/tof_sense_ms_parser.cpp
HEADERS  := ../src/drivers/distance_sensor/tof_sense_ms/tof_sense_ms_parser.h tof_sample_frame.h

.PHONY: all test clean

all: test_parser

test_parser: test_parser.cpp $(PARSER) $(HEADERS)
	$(CXX) $(CXXFLAGS) -I. -o $@ test_parser.cpp $(PARSER)

test: test_parser
	./test_parser

clean:
	rm -f test_parser
