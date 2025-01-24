// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/box_box_distance.hh"

#include <Eigen/Dense>

#include "resim/assert/assert.hh"
#include "resim/geometry/gjk_algorithm.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {

namespace {
using Vec3 = Eigen::Vector3d;
using transforms::LieGroupType;

// This helper represents the support function for a simple oriented box
template <LieGroupType Group>
Vec3 box_support(const OrientedBox<Group> &box, const Vec3 &direction) {
  REASSERT(not direction.isZero(), "Zero direction detected!");
  const Vec3 direction_in_box_coordinates{
      box.reference_from_box().rotation().inverse() * direction};

  const Vec3 support_in_box_coordinates{direction.binaryExpr(
      box.extents(),
      [](const double direction_element, const double extents_element) {
        constexpr double HALF = 0.5;
        constexpr double ZERO = 0.;
        return extents_element *
               (direction_element > 0.
                    ? HALF
                    : (direction_element < 0. ? -HALF : ZERO));
      })};

  Vec3 support_in_ref_coordinates{
      box.reference_from_box() * support_in_box_coordinates};
  return support_in_ref_coordinates;
}

}  // namespace

template <transforms::LieGroupType Group>
double box_box_distance(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2) {
  const bool frames_match =
      box_1.reference_from_box().into() == box_2.reference_from_box().into();
  REASSERT(frames_match, "Box frames don't match!");

  constexpr int DIM = 3;
  const SupportFunction<DIM> support_1{[&](const Vec3 &direction) -> Vec3 {
    return box_support(box_1, direction);
  }};
  const SupportFunction<DIM> support_2{[&](const Vec3 &direction) -> Vec3 {
    return box_support(box_2, direction);
  }};

  const std::optional<double> maybe_box_box_distance =
      gjk_algorithm(support_1, support_2);

  REASSERT(maybe_box_box_distance.has_value(), "GJK Algorithm Failed!");

  return *maybe_box_box_distance;
}

template double box_box_distance(
    const OrientedBox<transforms::SE3> &box_1,
    const OrientedBox<transforms::SE3> &box_2);

}  // namespace resim::geometry
