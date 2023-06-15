
#include "resim/geometry/wireframe.hh"

#include <algorithm>
#include <utility>

namespace resim::geometry {

namespace {

// This simple helper checks whether the given point_idx is within the
// bounds of the given array. The parent array type is templated so
// that we can handle faces that reference the indices of edges
// eventually.
// TODO(https://app.asana.com/0/1202178773526279/1203257064495359/f)
template <typename T>
bool index_in_parent(
    const std::size_t point_idx,
    const std::vector<T> &parent_array) {
  return point_idx < parent_array.size();
}

// This helper checks whether the given index array is completely
// contained within the given parent and contains unique elements. The
// parent array type and the index array size are templated so that we
// can handle faces that reference the indices of edges eventually.
// TODO(https://app.asana.com/0/1202178773526279/1203257064495359/f)
template <typename T, std::size_t N>
bool index_array_in_parent_and_unique(
    const std::array<std::size_t, N> &idx_array,
    const std::vector<T> &parent_array) {
  for (std::size_t ii = 0; ii < idx_array.size(); ++ii) {
    // Check for indices outside the points array
    if (not index_in_parent(idx_array.at(ii), parent_array)) {
      return false;
    }
    // Check for duplicates.
    for (std::size_t jj = ii + 1U; jj < idx_array.size(); ++jj) {
      if (idx_array.at(ii) == idx_array.at(jj)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

Wireframe::Wireframe(std::vector<Point> points, std::vector<Edge> edges)
    : points_{std::move(points)},
      edges_{std::move(edges)} {}

bool Wireframe::is_valid() const {
  return std::ranges::all_of(
      edges_.cbegin(),
      edges_.cend(),
      [this](const auto &edge) {
        return index_array_in_parent_and_unique(edge, points_);
      });
}

void Wireframe::add_point(const Point &point) { points_.push_back(point); }

void Wireframe::add_edge(const Edge &edge) { edges_.push_back(edge); }

const std::vector<Wireframe::Point> &Wireframe::points() const {
  return points_;
}

const std::vector<Wireframe::Edge> &Wireframe::edges() const { return edges_; }

}  // namespace resim::geometry
