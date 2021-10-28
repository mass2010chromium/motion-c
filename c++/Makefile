
CXXFLAGS = -I ../extern/pybind11/include $(shell python3-config --cflags) -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes)
PY_SUFFIX = $(shell python3-config --extension-suffix)

all:
	$(CXX) $(CFLAGS) rtde.cpp -o rtde$(PY_SUFFIX) $(CXXFLAGS)

