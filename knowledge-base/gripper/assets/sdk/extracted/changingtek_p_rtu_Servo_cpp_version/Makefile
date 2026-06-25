CXX = g++
CXXFLAGS = -std=c++11 -Wall -pthread
TARGET = example_servo
SRCS = main.cpp Changingtek_p_rtu_Servo.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
