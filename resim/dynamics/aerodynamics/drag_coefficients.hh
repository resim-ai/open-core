#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <utility>

namespace resim::dynamics::aerodynamics {

// Basic wrapper around Eigen::Vector3d for implementing specifically
// 3-DOF drag-coefficients. Specific accessor methods are included as
// the x/y/z() methods do not correspond to any particular geometric
// frame of reference.
//
// A vector is useful here primarily as there is a reasonable origin,
// and it makes sense to lerp these coefficients.
class DragCoefficients : public Eigen::Vector3d {
 public:
  DragCoefficients() = default;
  DragCoefficients(double lift_coeff, double drag_coeff, double pitch_coeff)
      : Eigen::Vector3d(lift_coeff, drag_coeff, pitch_coeff) {}

  explicit DragCoefficients(Eigen::Vector3d v)
      : Eigen::Vector3d(std::move(v)) {}
  double lift_coeff() const { return x(); }
  double drag_coeff() const { return y(); }
  double pitch_coeff() const { return z(); }

  DragCoefficients operator+(const DragCoefficients &other) const {
    return DragCoefficients(Eigen::Vector3d::operator+(other));
  }

  DragCoefficients operator*(double scalar) const {
    return DragCoefficients(Eigen::Vector3d::operator*(scalar));
  }

  friend DragCoefficients operator*(double scalar, const DragCoefficients &dc) {
    return DragCoefficients(dc.Eigen::Vector3d::operator*(scalar));
  }

  friend std::ostream &operator<<(
      std::ostream &os,
      const DragCoefficients &dc) {
    os << "DragCoefficients(lift_coeff: " << dc.lift_coeff()
       << ", drag_coeff: " << dc.drag_coeff()
       << ", pitch_coeff: " << dc.pitch_coeff() << ")";
    return os;
  }
};

}  // namespace resim::dynamics::aerodynamics
