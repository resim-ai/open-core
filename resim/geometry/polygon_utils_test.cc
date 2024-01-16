// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// NOLINTBEGIN(readability-magic-numbers)

#include "resim/geometry/polygon_utils.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "resim/testing/random_matrix.hh"

namespace resim::geometry {
using Vec2 = Eigen::Vector2d;

TEST(PolygonUtilsTest, TestParallelLines) {
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{0.0, 2.0}));

  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{-1.0, 1.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 2.0}));

  // Coincident lines
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
}

TEST(PolygonUtilsTest, TestDegenerateEdges) {
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 0.0},
      Vec2{0.0, 0.0}));
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{-1.0, 1.0}));
  EXPECT_FALSE(segment_intersection(
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0}));
}

TEST(PolygonUtilsTest, TestPastEnds) {
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{1.1 - 0.4, 1.1 + 0.4},
      Vec2{1.1 + 0.4, 1.1 - 0.4}));
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-0.1 - 0.4, -0.1 + 0.4},
      Vec2{-0.1 + 0.4, -0.1 - 0.4}));
  EXPECT_FALSE(segment_intersection(
      Vec2{1.1 - 0.4, 1.1 + 0.4},
      Vec2{1.1 + 0.4, 1.1 - 0.4},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
  EXPECT_FALSE(segment_intersection(
      Vec2{-0.1 - 0.4, -0.1 + 0.4},
      Vec2{-0.1 + 0.4, -0.1 - 0.4},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
}

TEST(PolygonUtilsTest, TestCoincidentEndpoint) {
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 2.0}));
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 0.0}));
}

TEST(PolygonUtilsTest, TestEndpointOnOtherSegment) {
  EXPECT_FALSE(segment_intersection(
      Vec2{0.0, 0.0},
      Vec2{2.0, 2.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 2.0}));
}

TEST(PolygonUtilsTest, TestIntersection) {
  EXPECT_TRUE(segment_intersection(
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, -1.0},
      Vec2{-1.0, 1.0}));
  EXPECT_TRUE(segment_intersection(
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{1.0, -1.0}));
}

TEST(PolygonUtilsTest, TestSelfIntersection) {
  EXPECT_TRUE(is_self_intersecting(std::vector{
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, -1.0},
      Vec2{-1.0, 1.0}}));

  // Try some regular polygons
  constexpr int MIN_EDGES = 5;
  constexpr int MAX_EDGES = 12;
  for (int ii = MIN_EDGES; ii < MAX_EDGES; ++ii) {
    std::vector<Vec2> polygon;
    polygon.reserve(ii);
    for (int jj = 0; jj < ii; ++jj) {
      polygon.emplace_back(
          std::cos(static_cast<double>(jj) / (ii - 1) / 2.0 / M_PI),
          std::sin(static_cast<double>(jj) / (ii - 1) / 2.0 / M_PI));
    }

    EXPECT_FALSE(is_self_intersecting(polygon));

    for (int jj = 0; jj < ii; ++jj) {
      for (int kk = jj + 1; kk < ii; ++kk) {
        std::vector<Vec2> flipped_polygon{polygon};
        std::swap(flipped_polygon.at(jj), flipped_polygon.at(kk));
        EXPECT_TRUE(is_self_intersecting(flipped_polygon));
      }
    }
  }

  EXPECT_TRUE(is_self_intersecting(std::vector{
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, -1.0},
      Vec2{-1.0, 1.0}}));
}

TEST(PolygonUtilsTest, TestPointSegmentDistance) {
  // SETUP
  constexpr size_t SEED = 4324U;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> dist{0.0, 1.0};

  constexpr double TOLERANCE = 1e-15;
  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Vec2 segment_start{testing::random_vector<Vec2>(rng)};
    const Vec2 segment_end{testing::random_vector<Vec2>(rng)};

    const Vec2 dir{segment_end - segment_start};
    const Vec2 normal{Vec2{-dir(1), dir(0)}.normalized()};

    // Edge closest:
    {
      const Vec2 segment_coords{testing::random_vector<Vec2>(rng, dist)};
      const Vec2 point{
          segment_start + dir * segment_coords(0) + normal * segment_coords(1)};

      // ACTION
      const double distance =
          point_line_segment_distance(point, segment_start, segment_end);

      // VERIFICATION
      EXPECT_LT(
          std::fabs(distance - segment_coords(1) /* normal coord */),
          TOLERANCE);
    }
    // Endpoint closest:
    {
      const Vec2 overshot_segment_coords{
          testing::random_vector<Vec2>(rng, dist) + Vec2::UnitX()};
      const Vec2 undershot_segment_coords{
          testing::random_vector<Vec2>(rng, dist) - Vec2::UnitX()};

      const Vec2 overshot_point{
          segment_start + dir * overshot_segment_coords(0) +
          normal * overshot_segment_coords(1)};
      const Vec2 undershot_point{
          segment_start + dir * undershot_segment_coords(0) +
          normal * undershot_segment_coords(1)};

      // ACTION
      const double overshot_distance = point_line_segment_distance(
          overshot_point,
          segment_start,
          segment_end);
      const double undershot_distance = point_line_segment_distance(
          undershot_point,
          segment_start,
          segment_end);

      // VERIFICATION
      EXPECT_LT(
          std::fabs(overshot_distance - (overshot_point - segment_end).norm()),
          TOLERANCE);
      EXPECT_LT(
          std::fabs(
              undershot_distance - (undershot_point - segment_start).norm()),
          TOLERANCE);
    }
  }
}

TEST(PolygonUtilsTest, TestPointSegmentDistanceOnSegment) {
  // SETUP
  constexpr double TOLERANCE = 1e-15;
  constexpr size_t SEED = 4324U;
  std::mt19937 rng{SEED};
  const Vec2 segment_start{testing::random_vector<Vec2>(rng)};
  const Vec2 segment_end{testing::random_vector<Vec2>(rng)};

  constexpr int NUM_POINTS = 10;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double frac = static_cast<double>(ii) / (NUM_POINTS - 1);
    const Vec2 point{(1. - frac) * segment_start + frac * segment_end};

    // ACTION / VERIFICATION
    EXPECT_LT(
        point_line_segment_distance(point, segment_start, segment_end),
        TOLERANCE);
  }
}

TEST(PolygonUtilsTest, TestPointSegmentDistanceOnDegenerate) {
  // SETUP
  constexpr double TOLERANCE = 1e-15;
  constexpr size_t SEED = 4324U;
  std::mt19937 rng{SEED};
  const Vec2 segment_start{testing::random_vector<Vec2>(rng)};
  const Vec2 segment_end{segment_start};

  constexpr int NUM_POINTS = 10;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const Vec2 point{testing::random_vector<Vec2>(rng)};
    // ACTION / VERIFICATION
    EXPECT_LT(
        point_line_segment_distance(point, segment_start, segment_end) -
            (point - segment_start).norm(),
        TOLERANCE);
  }
}

TEST(PolygonUtilsTest, TestPointInPolygon) {
  // SETUP
  const std::vector<Vec2> polygon{
      Vec2{-1.0, -1.0},
      Vec2{1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
  };

  // ACTION / VERIFICATION
  constexpr double EPS = 1e-12;
  EXPECT_TRUE(point_in_polygon(Vec2{0.0, 0.0}, polygon));
  EXPECT_TRUE(point_in_polygon(Vec2{1.0 - EPS, 0.0}, polygon));
  EXPECT_TRUE(point_in_polygon(Vec2{-1.0 + EPS, 0.0}, polygon));
  EXPECT_TRUE(point_in_polygon(Vec2{0.0, 1.0 - EPS}, polygon));
  EXPECT_TRUE(point_in_polygon(Vec2{0.0, -1.0 + EPS}, polygon));
}

TEST(PoygonUtilsTest, TestPointInNonConvexPolygon) {
  // SETUP
  // Make a spiral
  constexpr double OUTER_RADIUS = 20;
  constexpr double INNER_RADIUS = 18;
  const auto spiral = [](const double t, const double offset) -> Vec2 {
    const double angle_rad = 2. * M_PI * t;
    return (OUTER_RADIUS - t + offset) *
           Vec2{std::cos(angle_rad), std::sin(angle_rad)};
  };
  std::vector<Vec2> spiral_poly;
  constexpr int NUM_POINTS = 80;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double frac = static_cast<double>(ii) / (NUM_POINTS - 1);
    const double t = frac * (OUTER_RADIUS - INNER_RADIUS);
    constexpr double OFFSET = 0.2;
    spiral_poly.emplace_back(spiral(t, OFFSET));
  }
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double frac = 1.0 - static_cast<double>(ii) / (NUM_POINTS - 1);
    const double t = frac * (OUTER_RADIUS - INNER_RADIUS);
    constexpr double OFFSET = -0.2;
    spiral_poly.emplace_back(spiral(t, OFFSET));
  }

  ASSERT_TRUE(not is_self_intersecting(spiral_poly));

  // for (const auto &p : spiral_poly) {
  //   std::cout << p.transpose() << std::endl;
  // }

  // ACTION / VERIFICATION
  for (int ii = 0; ii < NUM_POINTS - 1; ++ii) {
    const double frac = (static_cast<double>(ii) + 0.5) / (NUM_POINTS - 1);
    const double t = frac * (OUTER_RADIUS - INNER_RADIUS);
    constexpr double IN_OFFSET = 0.0;
    constexpr double OUT_OFFSET = 0.5;
    EXPECT_TRUE(point_in_polygon(spiral(t, IN_OFFSET), spiral_poly));
    EXPECT_FALSE(point_in_polygon(spiral(t, OUT_OFFSET), spiral_poly));
  }
}

}  // namespace resim::geometry

// NOLINTEND(readability-magic-numbers)
