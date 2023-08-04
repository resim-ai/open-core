// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/transforms/frame.hh"

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

  LieGroup(Frame<dims> into, Frame<dims> from)
      : into_(std::move(into)),
        from_(std::move(from)) {
    const bool not_semi_framed = into_.is_null() == from_.is_null();
    constexpr auto SEMI_FRAMED_ERR =
        "We currently do not support semi-framed "
        "LieGroups. Neither or both frames can be null, but one null frame "
        "results in this error.";
    REASSERT(not_semi_framed, SEMI_FRAMED_ERR);
  }
  // Degrees of freedom of the LieGroup.
  static constexpr unsigned int DOF = dof;
  // Spatial Dimensionality of the LieGroup action.
  static constexpr unsigned int DIMS = dims;

  // For representing vectors in tangent space.
  using TangentVector = Eigen::Matrix<double, DOF, 1>;

  // For representing mappings between tangent spaces
  using TangentMapping = Eigen::Matrix<double, DOF, DOF>;

  // Set frames
  void set_frames(const Frame<DIMS> &into, const Frame<DIMS> &from) {
    constexpr auto NULL_FRAME_ERR =
        "Cannot set frames as null {0}. If you want to make this group "
        "unframed "
        "use set_unframed()";
    REASSERT(!(into.is_null() or from.is_null()), NULL_FRAME_ERR);
    into_ = into;
    from_ = from;
  }

  void set_unframed() {
    into_ = Frame<dims>();
    from_ = Frame<dims>();
  }

  // Retrieve the into Frame.
  const Frame<DIMS> &into() const { return into_; }

  // Retrieve the from Frame.
  const Frame<DIMS> &from() const { return from_; }

  // Is this liegroup strongly framed? Note, we check one because semi-unframed
  // should be impossible.
  bool is_framed() const { return !into_.is_null(); }

  // Test whether the into frame matches the user's expectation.
  bool verify_into(const Frame<DIMS> &candidate) const {
    return into_ == candidate;
  }

  // Test whether the from frame matches the user's expectation.
  bool verify_from(const Frame<DIMS> &candidate) const {
    return from_ == candidate;
  }

  // Test whether both frames match the user's expectation.
  bool verify_frames(
      const Frame<DIMS> &candidate_into,
      const Frame<DIMS> &candidate_from) const {
    return verify_into(candidate_into) && verify_from(candidate_from);
  }

 private:
  Frame<DIMS> into_;
  Frame<DIMS> from_;
};

}  // namespace resim::transforms
