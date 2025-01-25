// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/box_box_distance.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "oriented_box.hh"
#include "resim/assert/assert.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
namespace resim::geometry {

using Vec3 = Eigen::Vector3d;
using transforms::LieGroupType;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<transforms::SE3::DIMS>;

class BoxBoxDistanceTest : public ::testing::Test {};

TEST_F(BoxBoxDistanceTest, TestFaceFaceDistance) {
  // SETUP
  const Vec3 extents_a{Vec3{1.0, 2.0, 3.0}};
  const Vec3 extents_b{Vec3{3.0, 2.0, 1.0}};

  const SE3 scene_from_box_a{SE3::identity()};
  const SE3 scene_from_box_b{3.0 * Vec3::UnitY()};

  const OrientedBox<SE3> box_a{scene_from_box_a, extents_a};
  const OrientedBox<SE3> box_b{scene_from_box_b, extents_b};

  // ACTION
  const double distance_ab = box_box_distance(box_a, box_b);
  const double distance_ba = box_box_distance(box_b, box_a);

  // VERIFICATION
  constexpr double TOLERANCE = 1e-8;
  EXPECT_NEAR(
      distance_ab,
      scene_from_box_b.translation().y() - extents_a.y() / 2.0 -
          extents_b.y() / 2.0,
      TOLERANCE);
  EXPECT_NEAR(distance_ab, distance_ba, TOLERANCE);
}

TEST_F(BoxBoxDistanceTest, TestFaceEdgeDistance) {
  // SETUP
  const Vec3 extents_a{Vec3{1.0, 2.0, 3.0}};
  const Vec3 extents_b{Vec3{2.0, 2.0, 1.0}};

  const SE3 scene_from_box_a{SE3::identity()};
  constexpr double THETA = M_PI_4;
  const SE3 scene_from_box_b{
      SO3::exp(THETA * Vec3::UnitZ()),
      3.0 * Vec3::UnitY()};

  const OrientedBox<SE3> box_a{scene_from_box_a, extents_a};
  const OrientedBox<SE3> box_b{scene_from_box_b, extents_b};

  // ACTION
  const double distance_ab = box_box_distance(box_a, box_b);
  const double distance_ba = box_box_distance(box_b, box_a);

  // VERIFICATION
  constexpr double TOLERANCE = 1e-8;
  EXPECT_NEAR(
      distance_ab,
      scene_from_box_b.translation().y() - extents_a.y() / 2.0 -
          (extents_b.y() * std::cos(THETA) + extents_b.x() * std::sin(THETA)) /
              2.0,
      TOLERANCE);
  EXPECT_NEAR(distance_ab, distance_ba, TOLERANCE);
}

TEST_F(BoxBoxDistanceTest, TestFaceVertexDistance) {
  // SETUP
  const Vec3 extents_a{Vec3{1.0, 2.0, 3.0}};
  const Vec3 extents_b{Vec3{2.0, 2.0, 2.0}};

  // The idea is to find a rotation that maps a corner of our cube (for b) to be
  // as close to box a as possible. Find the rotation that maps (1, 1,
  // 1)/sqrt(3) to (0, -1, 0):
  const Vec3 source{Vec3::Ones() / std::sqrt(3.0)};
  const Vec3 dest{-Vec3::UnitY()};
  const Vec3 rot_angle_axis{
      source.cross(dest).normalized() * std::asin(source.cross(dest).norm())};

  const SE3 scene_from_box_a{SE3::identity()};
  const SE3 scene_from_box_b{SO3::exp(rot_angle_axis), 3.0 * Vec3::UnitY()};

  const OrientedBox<SE3> box_a{scene_from_box_a, extents_a};
  const OrientedBox<SE3> box_b{scene_from_box_b, extents_b};

  // ACTION
  const double distance_ab = box_box_distance(box_a, box_b);
  const double distance_ba = box_box_distance(box_b, box_a);

  // VERIFICATION
  // Based on the specific rotation we set up for b above and its extents:
  const double expected_distance =
      scene_from_box_b.translation().y() - extents_a.y() / 2.0 - std::sqrt(3.0);
  constexpr double TOLERANCE = 1e-8;
  EXPECT_NEAR(distance_ab, expected_distance, TOLERANCE);
  EXPECT_NEAR(distance_ab, distance_ba, TOLERANCE);
}

TEST_F(BoxBoxDistanceTest, TestEdgeEdgeDistance) {
  // SETUP
  const Vec3 extents_a{Vec3{2.0, 2.0, 3.0}};
  const Vec3 extents_b{Vec3{2.0, 2.0, 1.0}};

  constexpr double THETA = M_PI_4;
  const SE3 scene_from_box_a{
      SO3::exp(THETA * Vec3::UnitZ()),
  };
  const SE3 scene_from_box_b{
      SO3::exp(M_PI_2 * Vec3::UnitY()) * SO3::exp(THETA * Vec3::UnitZ()),
      3.0 * Vec3::UnitY()};

  const OrientedBox<SE3> box_a{scene_from_box_a, extents_a};
  const OrientedBox<SE3> box_b{scene_from_box_b, extents_b};

  // ACTION
  const double distance_ab = box_box_distance(box_a, box_b);
  const double distance_ba = box_box_distance(box_b, box_a);

  // VERIFICATION
  constexpr double TOLERANCE = 1e-8;
  EXPECT_NEAR(
      distance_ab,
      scene_from_box_b.translation().y() -
          (extents_a.y() * std::cos(THETA) + extents_a.x() * std::sin(THETA)) /
              2.0 -
          (extents_b.y() * std::cos(THETA) + extents_b.x() * std::sin(THETA)) /
              2.0,
      TOLERANCE);
  EXPECT_NEAR(distance_ab, distance_ba, TOLERANCE);
}

}  // namespace resim::geometry
