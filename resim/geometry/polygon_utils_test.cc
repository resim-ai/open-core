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
#include <utility>
#include <vector>

namespace resim::geometry {
using Vec2 = Eigen::Vector2d;

TEST(PolygonUtilsTest, TestParallelLines) {
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{0.0, 2.0}));

  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{-1.0, 1.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 2.0}));

  // Coincident lines
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
}

TEST(PolygonUtilsTest, TestDegenerateEdges) {
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{0.0, 0.0},
      Vec2{0.0, 0.0}));
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{-1.0, 1.0}));
  EXPECT_FALSE(edge_intersection(
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0},
      Vec2{1.0, -1.0}));
}

TEST(PolygonUtilsTest, TestPastEnds) {
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{1.1 - 0.4, 1.1 + 0.4},
      Vec2{1.1 + 0.4, 1.1 - 0.4}));
  EXPECT_FALSE(edge_intersection(
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0},
      Vec2{-0.1 - 0.4, -0.1 + 0.4},
      Vec2{-0.1 + 0.4, -0.1 - 0.4}));
  EXPECT_FALSE(edge_intersection(
      Vec2{1.1 - 0.4, 1.1 + 0.4},
      Vec2{1.1 + 0.4, 1.1 - 0.4},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
  EXPECT_FALSE(edge_intersection(
      Vec2{-0.1 - 0.4, -0.1 + 0.4},
      Vec2{-0.1 + 0.4, -0.1 - 0.4},
      Vec2{0.0, 0.0},
      Vec2{1.0, 1.0}));
}

TEST(PolygonUtilsTest, TestIntersection) {
  EXPECT_TRUE(edge_intersection(
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, -1.0},
      Vec2{-1.0, 1.0}));
  EXPECT_TRUE(edge_intersection(
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{-1.0, 1.0},
      Vec2{1.0, -1.0}));
}

TEST(PolygonUtilsTest, TestSelfIntersection) {
  EXPECT_TRUE(self_intersecting(std::vector{
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

    EXPECT_FALSE(self_intersecting(polygon));

    for (int jj = 0; jj < ii; ++jj) {
      for (int kk = jj + 1; kk < ii; ++kk) {
        std::vector<Vec2> flipped_polygon{polygon};
        std::swap(flipped_polygon.at(jj), flipped_polygon.at(kk));
        EXPECT_TRUE(self_intersecting(flipped_polygon));
      }
    }
  }

  EXPECT_TRUE(self_intersecting(std::vector{
      Vec2{-1.0, -1.0},
      Vec2{1.0, 1.0},
      Vec2{1.0, -1.0},
      Vec2{-1.0, 1.0}}));
}

}  // namespace resim::geometry

// NOLINTEND(readability-magic-numbers)
