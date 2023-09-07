
#include "resim/geometry/polygon.hh"

namespace resim::geometry {

double compute_area(const Polygon &polygon) {
  // https://en.wikipedia.org/wiki/Shoelace_formula
  double area = 0.;
  for (std::size_t ii = 0; ii < polygon.vertices.size(); ++ii) {
    std::size_t next_idx = (ii + 1U) % polygon.vertices.size();
    const Eigen::Vector2d &p1{polygon.vertices.at(ii)};
    const Eigen::Vector2d &p2{polygon.vertices.at(next_idx)};
    area += p1.x() * p2.y() - p2.x() * p1.y();
  }
  // NOLINTNEXTLINE(readability-magic-numbers)
  return area / 2.;
}

}  // namespace resim::geometry
