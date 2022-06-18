#pragma once

#include <Eigen/Dense>

namespace resim {
namespace transforms {

// Abstract base class for LieGroups.
// All LieGroup classes (e.g. SE3, SO3) represent a rigid transformation in
// *dims* dimensional space with *dof* degrees of freedom. Elements of the
// LieGroup algebra are represented by dofx1 vectors in tangent space, defined
// here as the LieGroup::TangentVector type.

// For compatibility with other classes that are templated with a LieGroup
// typename a valid LieGroup class should:
//   1. Inherit from this class.
//   2. Implement all the pure virtual methods of this class.
//   3. Implement the following static methods:
//     a. static Group identity(); // Get a noop transform.
//     b. // Construct a Group from and algebra element.
//        static Group exp(const TangentVector & alg);
template <typename Group, const unsigned int dims, const unsigned int dof>
class LieGroup {
 public:
  // Degrees of freedom of the LieGroup.
  static constexpr unsigned int DOF = dof;
  // Spatial Dimensionality of the LieGroup action.
  static constexpr unsigned int DIMS = dims;

  // For representing vectors in tangent space.
  using TangentVector = Eigen::Matrix<double, DOF, 1>;

  // Compose the Group with another (multiplication)
  virtual Group operator*(const Group &other) const = 0;

  // Apply the action of the Group on a vector.
  virtual Eigen::Matrix<double, DIMS, 1> operator*(
      const Eigen::Matrix<double, DIMS, 1> &source_vector) const = 0;

  // Get the Group representing the inverse transform of this.
  virtual Group inverse() const = 0;

  // Interpolate the transform.
  // [param] fraction - interpolation is over a unit interval, where
  // fraction=0 returns identity and fraction=1 returns this Group. In between
  // the Group returned is a linear interpolation. If fraction is greater than 1
  // or less than 0, a linear extrapolation will be returned.
  virtual Group interp(const double fraction) const = 0;

  // Retrieve the element of the LieGroup algebra that represents
  // this group element.
  virtual TangentVector log() const = 0;

  // Transform a TangentVector from the right tangent space to the left.
  virtual TangentVector adjoint_times(const TangentVector &alg) const = 0;

  // Test for floating-point equality with another Group.
  virtual bool is_approx(const Group &other) const = 0;
};

}  // namespace transforms
}  // namespace resim
