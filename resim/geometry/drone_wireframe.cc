
#include "resim/geometry/drone_wireframe.hh"

#include <cmath>

#include "resim/assert/assert.hh"

namespace resim::geometry {

Wireframe drone_wireframe(const DroneExtents &drone_extents) {
  constexpr auto ERROR_MESSAGE = "Invalid drone extents!";
  REASSERT(drone_extents.chassis_radius_m > 0., ERROR_MESSAGE);
  REASSERT(drone_extents.rotor_radius_m > 0., ERROR_MESSAGE);
  REASSERT(drone_extents.samples_per_rotor > 1, ERROR_MESSAGE);

  using Point = Wireframe::Point;
  using Edge = Wireframe::Edge;
  std::vector<Point> points;
  std::vector<Edge> edges;

  constexpr std::size_t NUM_ROTORS = 4;
  // Four per arm
  constexpr std::size_t NUM_CHASSIS_POINTS = 8;
  // Three per arm plus one to make the direction clear
  constexpr std::size_t NUM_CHASSIS_EDGES = 7;

  const std::size_t num_points =
      NUM_ROTORS * drone_extents.samples_per_rotor + NUM_CHASSIS_POINTS;
  const std::size_t num_edges =
      NUM_ROTORS * drone_extents.samples_per_rotor + NUM_CHASSIS_EDGES;

  points.reserve(num_points);
  edges.reserve(num_edges);

  // Setup the chassis
  {
    const std::size_t start_index = points.size();

    const double r = drone_extents.chassis_radius_m;
    const double h_l = drone_extents.rotor_lateral_offset_m;
    const double h_v = drone_extents.rotor_vertical_offset_m;

    points.emplace_back(-(r + h_l) * M_SQRT1_2, -(r + h_l) * M_SQRT1_2, h_v);
    points.emplace_back(-r * M_SQRT1_2, -r * M_SQRT1_2, 0.0);
    points.emplace_back(r * M_SQRT1_2, r * M_SQRT1_2, 0.0);
    points.emplace_back((r + h_l) * M_SQRT1_2, (r + h_l) * M_SQRT1_2, h_v);

    points.emplace_back(-(r + h_l) * M_SQRT1_2, (r + h_l) * M_SQRT1_2, h_v);
    points.emplace_back(-r * M_SQRT1_2, r * M_SQRT1_2, 0.0);
    points.emplace_back(r * M_SQRT1_2, -r * M_SQRT1_2, 0.0);
    points.emplace_back((r + h_l) * M_SQRT1_2, -(r + h_l) * M_SQRT1_2, h_v);

    // NOLINTBEGIN(readability-magic-numbers)
    // First chassis arm
    edges.emplace_back(Edge{start_index, start_index + 1U});
    edges.emplace_back(Edge{start_index + 1U, start_index + 2U});
    edges.emplace_back(Edge{start_index + 2U, start_index + 3U});
    // Second chassis arm
    edges.emplace_back(Edge{start_index + 4U, start_index + 5U});
    edges.emplace_back(Edge{start_index + 5U, start_index + 6U});
    edges.emplace_back(Edge{start_index + 6U, start_index + 7U});
    // Cross beam
    edges.emplace_back(Edge{start_index + 1U, start_index + 5U});
    // NOLINTEND(readability-magic-numbers)
  }

  // Setup the rotors
  {
    for (const std::size_t index : {0U, 3U, 4U, 7U}) {
      const Point center{points.at(index)};
      const std::size_t start_index = points.size();

      for (std::size_t ii = 0; ii < drone_extents.samples_per_rotor; ++ii) {
        const double theta_rad =
            (2.0 * M_PI * static_cast<double>(ii)) /
            static_cast<double>(drone_extents.samples_per_rotor);

        const Point current_point{
            center + drone_extents.rotor_radius_m *
                         Point{std::cos(theta_rad), std::sin(theta_rad), 0.0}};
        points.emplace_back(current_point);

        if (ii > 0) {
          edges.emplace_back(Edge{start_index + ii - 1U, start_index + ii});
        }
      }
      // Close the loop
      edges.emplace_back(Edge{points.size() - 1U, start_index});
    }
  }

  Wireframe result{std::move(points), std::move(edges)};
  REASSERT(result.is_valid(), "Drone wireframe is not valid!");
  return result;
}

}  // namespace resim::geometry
