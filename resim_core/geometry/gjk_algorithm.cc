
#include "resim_core/geometry/gjk_algorithm.hh"

#include <limits>
#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/geometry/gjk_distance_subalgorithm.hh"

namespace resim::geometry {

template <int DIM>
std::optional<double> gjk_algorithm(
    const SupportFunction<DIM> &support_1,
    const SupportFunction<DIM> &support_2,
    const int max_iterations) {
  using Point = Eigen::Matrix<double, DIM, 1>;
  using Vector = Point;

  const SupportFunction<DIM> support_difference{
      [&](const Vector &direction) -> Point {
        return support_1(direction) - support_2(-direction);
      }};

  double best_norm = std::numeric_limits<double>::max();
  const auto has_converged = [&best_norm](const Point &point) -> double {
    // Special case so that we don't pass zero into the support.
    if (point.isZero()) {
      return true;
    }

    // Terminate once we're no longer getting better
    const double current_norm = point.norm();
    if (best_norm <= current_norm) {
      return true;
    }
    best_norm = current_norm;
    return false;
  };

  // Initialize the simplex
  convex_helpers::Simplex<DIM> simplex{support_difference(Vector::Unit(0))};

  for (int ii = 0; ii < max_iterations; ++ii) {
    const convex_helpers::DistanceResult distance_result{
        convex_helpers::distance_subalgorithm(simplex)};
    simplex = std::move(distance_result.simplex);

    if (has_converged(distance_result.closest_point)) {
      return distance_result.closest_point.norm();
    }

    // If the simplex size is greater than DIM, then that means the simplex
    // closest to the origin is either a triangle in 2D or a tetrahedron in
    // 3D. For the entire triangle/tetrahedron to be the closest simplex (rather
    // than one of its boundary vertices/edges/faces), it must contain the
    // origin, meaning we should have already converged and exited before we
    // reach here. We accordingly check that ths simplex is *not* greater than
    // size DIM so that we don't get an invalid simplex when we add the next
    // point to it.
    constexpr auto ERROR_MESSAGE =
        "This simplex should have intersected the origin!";
    REASSERT(simplex.size() <= DIM, ERROR_MESSAGE);
    simplex.push_back(support_difference(-distance_result.closest_point));
  }
  return std::nullopt;
}

template std::optional<double> gjk_algorithm(
    const SupportFunction<3> &support_1,
    const SupportFunction<3> &support_2,
    int max_iterations);

template std::optional<double> gjk_algorithm(
    const SupportFunction<2> &support_1,
    const SupportFunction<2> &support_2,
    int max_iterations);

}  // namespace resim::geometry
