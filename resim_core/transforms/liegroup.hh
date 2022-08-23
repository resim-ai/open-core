#pragma once

#include <Eigen/Dense>

namespace resim::transforms {

// Abstract base class for LieGroups.
// All LieGroup classes (e.g. SE3, SO3) represent a rigid transformation in
// *dims* dimensional space with *dof* degrees of freedom. Elements of the
// LieGroup algebra are represented by dofx1 vectors in tangent space, defined
// here as the LieGroup::TangentVector type.

// For compatibility with other classes, a valid LieGroup class should inherit
// from this class.
template <unsigned int dims, unsigned int dof>
class LieGroup {
 public:
  LieGroup() = default;
  LieGroup(const LieGroup &) = default;
  LieGroup(LieGroup &&) noexcept = default;
  LieGroup &operator=(LieGroup &&) noexcept = default;
  LieGroup &operator=(const LieGroup &) = default;
  virtual ~LieGroup() = default;

  // Degrees of freedom of the LieGroup.
  static constexpr unsigned int DOF = dof;
  // Spatial Dimensionality of the LieGroup action.
  static constexpr unsigned int DIMS = dims;

  // For representing vectors in tangent space.
  using TangentVector = Eigen::Matrix<double, DOF, 1>;

  // For representing mappings between tangent spaces
  using TangentMapping = Eigen::Matrix<double, DOF, DOF>;
};

}  // namespace resim::transforms
