
#include "resim/geometry/polygon.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::geometry {

TEST(PolygonTest, TestAreaTriangles) {
  // SETUP
  using Vec2 = Eigen::Vector2d;

  constexpr double LOWER_BOUND = 0.;
  constexpr double UPPER_BOUND = 10.0;
  std::uniform_real_distribution<double> dist{LOWER_BOUND, UPPER_BOUND};

  constexpr std::size_t SEED = 74U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Vec2 vertex_1{testing::random_matrix<Vec2>(rng)};
    const Vec2 vertex_2{testing::random_matrix<Vec2>(rng)};
    const Vec2 dir{(vertex_2 - vertex_1).normalized()};
    const Vec2 normal{-dir.y(), dir.x()};

    const double base = (vertex_2 - vertex_1).norm();
    const double height = dist(rng);

    const Vec2 vertex_3{vertex_1 + height * normal + dir * dist(rng)};

    const Polygon permutation_0{{vertex_1, vertex_2, vertex_3}};
    const Polygon permutation_1{{vertex_2, vertex_3, vertex_1}};
    const Polygon permutation_2{{vertex_3, vertex_1, vertex_2}};
    const Polygon permutation_3{{vertex_2, vertex_1, vertex_3}};
    const Polygon permutation_4{{vertex_1, vertex_3, vertex_2}};
    const Polygon permutation_5{{vertex_3, vertex_2, vertex_1}};

    const double expected_area = 0.5 * base * height;

    // ACTION / VERIFICATION
    EXPECT_TRUE(math::is_approx(compute_area(permutation_0), expected_area));
    EXPECT_TRUE(math::is_approx(compute_area(permutation_1), expected_area));
    EXPECT_TRUE(math::is_approx(compute_area(permutation_2), expected_area));
    EXPECT_TRUE(math::is_approx(compute_area(permutation_3), -expected_area));
    EXPECT_TRUE(math::is_approx(compute_area(permutation_4), -expected_area));
    EXPECT_TRUE(math::is_approx(compute_area(permutation_5), -expected_area));
  }
}

TEST(PolygonTest, TestAreaRectangles) {
  // SETUP
  constexpr double LOWER_BOUND = 0.;
  constexpr double UPPER_BOUND = 10.0;
  std::uniform_real_distribution<double> dist{LOWER_BOUND, UPPER_BOUND};

  constexpr std::size_t SEED = 74U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const double rectangle_width = dist(rng);
    const double rectangle_height = dist(rng);
    const double rectangle_x = dist(rng);
    const double rectangle_y = dist(rng);
    const double rectangle_theta = dist(rng);

    const auto map_point = [&](const double x,
                               const double y) -> Eigen::Vector2d {
      return {
          x * std::cos(rectangle_theta) + y * std::sin(rectangle_theta) +
              rectangle_x,
          y * std::cos(rectangle_theta) - x * std::sin(rectangle_theta) +
              rectangle_y};
    };

    const Polygon rectangle_oriented{
        .vertices =
            {
                map_point(0, 0),
                map_point(rectangle_width, 0),
                map_point(rectangle_width, rectangle_height),
                map_point(0, rectangle_height),
            },
    };

    Polygon rectangle_reversed{rectangle_oriented};
    std::reverse(
        rectangle_reversed.vertices.begin(),
        rectangle_reversed.vertices.end());

    // ACTION
    const double area = compute_area(rectangle_oriented);
    const double negative_area = compute_area(rectangle_reversed);

    // VERIFICATION
    const double expected_area = rectangle_width * rectangle_height;
    EXPECT_TRUE(math::is_approx(area, expected_area));
    EXPECT_TRUE(math::is_approx(-negative_area, expected_area));
  }
}

}  // namespace resim::geometry
