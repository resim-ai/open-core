#include "resim/geometry/bounding_box_from_wireframe.hh"

#include <Eigen/Dense>

#include "resim/assert/assert.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {
using transforms::SE3;
using Frame = transforms::Frame<transforms::SE3::DIMS>;
using Vec3 = Eigen::Vector3d;

OrientedBox<SE3> bounding_box_from_wireframe(const Wireframe &wireframe) {
  return bounding_box_from_wireframe(
      wireframe,
      Frame::null_frame(),
      Frame::null_frame());
}

OrientedBox<SE3> bounding_box_from_wireframe(
    const Wireframe &wireframe,
    const Frame &reference_frame,
    const Frame &box_frame) {
  constexpr auto EMPTY_WIREFRAME_MSG =
      "Bounding box can't be found for empty or size one wireframe!";
  REASSERT(wireframe.points().size() > 1U, EMPTY_WIREFRAME_MSG);
  Vec3 max_vector{
      Vec3::NullaryExpr([]() { return -std::numeric_limits<double>::max(); })};
  Vec3 min_vector{
      Vec3::NullaryExpr([]() { return std::numeric_limits<double>::max(); })};

  for (const auto &point : wireframe.points()) {
    for (int ii = 0; ii < point.rows(); ++ii) {
      const double entry = point(ii);
      if (entry > max_vector(ii)) {
        max_vector(ii) = entry;
      }
      if (entry < min_vector(ii)) {
        min_vector(ii) = entry;
      }
    }
  }

  const Vec3 box_translation{0.5 * max_vector + 0.5 * min_vector};
  const Vec3 box_extents = max_vector - min_vector;
  constexpr auto ZERO_EXTENT_MSG = "Wireframe has at least one zero extent!";
  REASSERT((box_extents.array() > 0.).all(), ZERO_EXTENT_MSG);
  return OrientedBox{
      SE3{box_translation, reference_frame, box_frame},
      box_extents};
}

}  // namespace resim::geometry
