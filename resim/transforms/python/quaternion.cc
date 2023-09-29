#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of Eigen::Quaterniond
PYBIND11_MODULE(quaternion, m) {
  py::class_<Eigen::Quaterniond>(m, "Quaternion")
      .def(py::init<const Eigen::Vector4d &>())
      .def(
          py::init<double, double, double, double>(),
          py::arg("w"),
          py::arg("x"),
          py::arg("y"),
          py::arg("z"))
      .def("w", py::overload_cast<>(&Eigen::Quaterniond::w))
      .def("x", py::overload_cast<>(&Eigen::Quaterniond::x))
      .def("y", py::overload_cast<>(&Eigen::Quaterniond::y))
      .def("z", py::overload_cast<>(&Eigen::Quaterniond::z));
}

}  // namespace resim::transforms
