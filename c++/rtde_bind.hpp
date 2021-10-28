#include <string>
#include <pybind11/pybind11.h>

using std::string;

namespace py = pybind11;

PYBIND11_MODULE(rtde, m) {
    py::class_<RTDE>(m, "RTDE")
        .def(py::init<const string&, const int&>())
        .def("connect", &RTDE::connect);
}
