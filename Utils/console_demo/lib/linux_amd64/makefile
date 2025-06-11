CXX = g++
CXXFLAGS = -std=c++11
INCLUD_DIR = ./include
LIBS_DIR = ./lib
EXECUTABLE = KWTestApp
SRCS = main.cpp

all: $(EXECUTABLE)

$(EXECUTABLE): main.cpp
	$(CXX) -o $@ $^ -I$(INCLUD_DIR) -L$(LIBS_DIR) -lkw-cpp-lib -lpthread -ldl -Wl,-rpath,$(LIBS_DIR)


clean:
	rm -rf build ./KWTestApp

.PHONY: all clean