
#include "resim_core/geometry/proto/wireframe_to_proto.hh"

#include <Eigen/Dense>
#include <utility>
#include <vector>

#include "resim_core/assert/assert.hh"
#include "resim_core/geometry/wireframe.hh"
#include "resim_core/math/proto/matrix_to_proto.hh"
#include "resim_core/utils/inout.hh"

namespace resim::geometry::proto {

void pack(const geometry::Wireframe &in, Wireframe *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  REASSERT(in.is_valid(), "Can't pack invalid wireframe!");

  for (const Eigen::Vector3d &point : in.points()) {
    math::proto::pack_matrix(point, out->add_points()->mutable_values());
  }
  for (const auto &[start, end] : in.edges()) {
    auto &edge = *out->add_edges();
    edge.set_start(start);
    edge.set_end(end);
  }
}

geometry::Wireframe unpack(const Wireframe &in) {
  std::vector<geometry::Wireframe::Point> points;
  std::vector<geometry::Wireframe::Edge> edges;

  points.reserve(in.points_size());
  for (int ii = 0; ii < in.points_size(); ++ii) {
    Eigen::Vector3d point;
    math::proto::unpack_matrix(in.points(ii).values(), InOut{point});
    points.push_back(point);
  }
  edges.reserve(in.edges_size());
  for (int ii = 0; ii < in.edges_size(); ++ii) {
    edges.push_back({in.edges(ii).start(), in.edges(ii).end()});
  }

  geometry::Wireframe result{std::move(points), std::move(edges)};
  REASSERT(result.is_valid(), "Can't unpack invalid wireframe!");
  return result;
}

}  // namespace resim::geometry::proto
