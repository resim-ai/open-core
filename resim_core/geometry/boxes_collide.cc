
#include "resim_core/geometry/boxes_collide.hh"

#include <glog/logging.h>

#include <Eigen/Dense>

#include "resim_core/geometry/gjk_algorithm.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::geometry {

namespace {
using Vec3 = Eigen::Vector3d;
using transforms::LieGroupType;

// This helper represents the support function for a simple oriented box
template <LieGroupType Group>
Vec3 box_support(const OrientedBox<Group> &box, const Vec3 &direction) {
  CHECK(not direction.isZero()) << "Zero direction detected!";
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

template <LieGroupType Group>
bool boxes_collide(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2,
    const double collision_tolerance) {
  if constexpr (transforms::FramedGroupType<Group>) {
    const bool frames_match =
        box_1.reference_from_box().into() == box_2.reference_from_box().into();
    CHECK(frames_match) << "Box frames don't match!";
  }
  constexpr int DIM = 3;
  const SupportFunction<DIM> support_1{[&](const Vec3 &direction) -> Vec3 {
    return box_support(box_1, direction);
  }};
  const SupportFunction<DIM> support_2{[&](const Vec3 &direction) -> Vec3 {
    return box_support(box_2, direction);
  }};

  const std::optional<double> maybe_box_box_distance =
      gjk_algorithm(support_1, support_2);

  CHECK(maybe_box_box_distance.has_value()) << "GJK Algorithm Failed!";

  return *maybe_box_box_distance < collision_tolerance;
}

template bool boxes_collide(
    const OrientedBox<transforms::SE3> &box_1,
    const OrientedBox<transforms::SE3> &box_2,
    double collision_tolerance);

template bool boxes_collide(
    const OrientedBox<transforms::FSE3> &box_1,
    const OrientedBox<transforms::FSE3> &box_2,
    double collision_tolerance);

}  // namespace resim::geometry
