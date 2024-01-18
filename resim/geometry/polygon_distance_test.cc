// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/polygon_distance.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "resim/assert/assert.hh"

namespace resim::geometry {

TEST(PolygonDistanceTest, TestOverlappingTriangles) {
  // SETUP
  const std::vector<Eigen::Vector2d> triangle_a{
      {0.0, 1.0},
      {1.0, -0.5},
      {-1.0, -0.5},
  };
  const std::vector<Eigen::Vector2d> triangle_b{
      {0.0, -1.0},
      {1.0, 0.5},
      {-1.0, 0.5},
  };

  // ACTION
  const double distance = polygon_distance(triangle_a, triangle_b);

  // VERIFICATION
  EXPECT_EQ(distance, 0.0);
}

TEST(PolygonDistanceTest, TestNonoverlappingTriangles) {
  // SETUP
  const std::vector<Eigen::Vector2d> triangle_a{
      {0.0, 1.0},
      {1.0, -0.5},
      {-1.0, -0.5},
  };
  const std::vector<Eigen::Vector2d> triangle_b{
      {0.0, 1.5},
      {1.0, 2.5},
      {-1.0, 2.5},
  };

  // ACTION
  const double distance = polygon_distance(triangle_a, triangle_b);

  // VERIFICATION
  EXPECT_EQ(distance, 0.5);
}

TEST(PolygonDistanceTest, TestSelfIntersection) {
  // SETUP
  const std::vector<Eigen::Vector2d> polygon_a{
      {1.0, -0.5},
      {-1.0, -0.5},
      {1.0, 0.5},
      {-1.0, 0.5},
  };
  const std::vector<Eigen::Vector2d> triangle_b{
      {0.0, 1.5},
      {1.0, 2.5},
      {-1.0, 2.5},
  };

  // ACTION / VERIFICATION
  EXPECT_THROW(polygon_distance(polygon_a, triangle_b), AssertException);
}

TEST(PolygonDistanceTest, TestNonConvex) {
  // SETUP
  const std::vector<Eigen::Vector2d> polygon_a{
      {-1.0, 1.0},
      {-1.0, -1.0},
      {0.5, 0.5},
      {1.0, -1.0},
  };
  const std::vector<Eigen::Vector2d> triangle_b{
      {0.0, 1.5},
      {1.0, 2.5},
      {-1.0, 2.5},
  };

  // ACTION / VERIFICATION
  EXPECT_THROW(polygon_distance(polygon_a, triangle_b), AssertException);
}

TEST(PolygonDistanceTest, TestOverlappingRectangles) {
  // SETUP
  const std::vector<Eigen::Vector2d> polygon_a{
      {-1.0, -100.0},
      {-1.0, 100.0},
      {1.0, 100.0},
      {1.0, -100.0},
  };
  const std::vector<Eigen::Vector2d> polygon_b{
      {-100.0, -1.0},
      {-100.0, 1.0},
      {100.0, 1.0},
      {100.0, -1.0},
  };

  // ACTION / VERIFICATION
  EXPECT_EQ(0., polygon_distance(polygon_a, polygon_b));
}

TEST(PolygonDistanceTest, TestSinglePoint) {
  // SETUP
  const std::vector<Eigen::Vector2d> polygon_a{
      {0.0, 1.0},
  };
  const std::vector<Eigen::Vector2d> polygon_b{
      {0.0, 1.5},
      {1.0, 2.5},
  };

  // ACTION / VERIFICATION
  EXPECT_EQ(polygon_distance(polygon_a, polygon_b), 0.5);
}

}  // namespace resim::geometry
