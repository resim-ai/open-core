#include "resim_core/geometry/bounding_box_from_wireframe.hh"

#include <Eigen/Dense>

#include "glog/logging.h"

namespace resim::geometry {
using transforms::FSE3;
using transforms::SE3;
using Frame3 = transforms::Frame<3>;
using Vec3 = Eigen::Vector3d;

OrientedBox<SE3> bounding_box_from_wireframe(const Wireframe &wireframe) {
  constexpr auto EMPTY_WIREFRAME_MSG =
      "Bounding box can't be found for empty or size one wireframe!";
  CHECK(wireframe.points().size() > 1U) << EMPTY_WIREFRAME_MSG;
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
  CHECK((box_extents.array() > 0.).all()) << ZERO_EXTENT_MSG;
  return OrientedBox{SE3{box_translation}, box_extents};
}

OrientedBox<FSE3> bounding_box_from_wireframe(
    const Wireframe &wireframe,
    const Frame3 &reference_frame,
    const Frame3 &box_frame) {
  const auto unframed_box = bounding_box_from_wireframe(wireframe);
  return OrientedBox<FSE3>{
      FSE3{unframed_box.reference_from_box(), reference_frame, box_frame},
      unframed_box.extents()};
}

}  // namespace resim::geometry
