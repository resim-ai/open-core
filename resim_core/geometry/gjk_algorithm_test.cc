
#include "resim_core/geometry/gjk_algorithm.hh"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <utility>

#include "resim_core/geometry/oriented_box.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::geometry {
namespace {
template <int DIM>
using Point = Eigen::Matrix<double, DIM, 1>;

template <int DIM>
using Vector = Point<DIM>;

using Vec3 = Eigen::Vector3d;

using transforms::LieGroupType;
using transforms::SE3;
using transforms::SO3;

constexpr double ZERO = 0.;
constexpr double ONE = 1.;

// A small floating point number used to test both sides of the collision
// envelope.
constexpr double SMALL = 1e-3;

// A numerical tolerance.
constexpr double TOLERANCE = 1.e-14;

}  // namespace

// A helper function which generates a sphere support function with the given
// center and radius.
template <int DIM>
SupportFunction<DIM> sphere_support(Point<DIM> center, const double radius) {
  return [radius, center = std::move(center)](
             const Vector<DIM> &direction) -> Point<DIM> {
    return center + radius * direction.normalized();
  };
}

// A helper function which generates a box support function for the given
// oriented box.
template <LieGroupType Group>
SupportFunction<3> box_support(OrientedBox<Group> box) {
  return [box = std::move(box)](const Vector<3> &direction) -> Point<3> {
    CHECK(not direction.isZero()) << "Invalid direction!";

    return box.reference_from_box() *
           (box.reference_from_box().rotation().inverse() *
            direction.template cast<double>())
               .binaryExpr(box.extents(), [](const double x, const double l) {
                 constexpr double HALF = 0.5;
                 return l * (x > 0. ? HALF : -HALF);
               });
  };
}

// Generate a random oriented box with a given translation, and random
// orientation and extents.
template <typename Rng>
OrientedBox<SE3> random_box(Vector<3> translation, Rng &&rng) {
  std::uniform_real_distribution<double> ang_dist{-M_PI_2, M_PI_2};
  constexpr double LOWER_BOUND = 0.1;
  constexpr double UPPER_BOUND = 1.0;
  std::uniform_real_distribution<double> extents_dist{LOWER_BOUND, UPPER_BOUND};
  return {
      SE3{SO3::exp(testing::random_vector<SO3::TangentVector>(rng, ang_dist)),
          std::move(translation)},
      testing::random_vector<Eigen::Vector3d>(rng, extents_dist)};
}

////////////////////////////////////////////////////////////////////////////////
// 2D Tests
////////////////////////////////////////////////////////////////////////////////

// Test that we can find the distance between two circles in 2D when they do or
// do not intersect.
TEST(GJKAlgorithmTest, Test2DCircles) {
  // SETUP
  constexpr double RADIUS = M_SQRT1_2;

  // A and B should intersect, B and C should barely not intersect, and A and C
  // should be far apart.
  const auto support_a{sphere_support<2>({ONE, ONE}, RADIUS + SMALL)};
  const auto support_b{sphere_support<2>({ZERO, ZERO}, RADIUS)};
  const auto support_c{sphere_support<2>({-ONE, -ONE}, RADIUS - SMALL)};

  // ACTION
  const double distance_ab = *gjk_algorithm(support_a, support_b);
  const double distance_bc = *gjk_algorithm(support_b, support_c);
  const double distance_ac = *gjk_algorithm(support_a, support_c);

  // VERIFICATION
  EXPECT_NEAR(distance_ab, ZERO, TOLERANCE);
  EXPECT_NEAR(distance_bc, SMALL, TOLERANCE);
  EXPECT_NEAR(distance_ac, 2. * (M_SQRT2 - RADIUS), TOLERANCE);
}

// Test that we fail when no convergence occurs.
TEST(GJKAlgorithmTest, NoConvergence2D) {
  // SETUP
  const auto support_a{
      sphere_support<2>(Eigen::Matrix<double, 2, 1>::Ones(), ONE)};
  const auto support_b{
      sphere_support<2>(-Eigen::Matrix<double, 2, 1>::Ones(), ONE)};

  constexpr int MAX_ITERATIONS = 1U;

  // ACTION / VERIFICATION
  EXPECT_FALSE(gjk_algorithm(support_a, support_b, MAX_ITERATIONS).has_value());
}

////////////////////////////////////////////////////////////////////////////////
// 3D Tests
////////////////////////////////////////////////////////////////////////////////

// Test that we can find the distance between two spheres in 3D when they do or
// do not intersect.
TEST(GJKAlgorithmTest, Test3DCircles) {
  // SETUP
  const double radius = std::sqrt(3.) / 2.0;

  // A and B should intersect, B and C should barely not intersect, and A and C
  // should be far apart.
  using Vec3Type = Eigen::Matrix<double, 3, 1>;
  const auto support_a{sphere_support<3>(Vec3Type::Ones(), radius + SMALL)};
  const auto support_b{sphere_support<3>(Vec3Type::Zero(), radius)};
  const auto support_c{sphere_support<3>(-Vec3Type::Ones(), radius - SMALL)};

  // ACTION
  const double distance_ab = *gjk_algorithm(support_a, support_b);
  const double distance_bc = *gjk_algorithm(support_b, support_c);
  const double distance_ac = *gjk_algorithm(support_a, support_c);

  // VERIFICATION
  EXPECT_NEAR(distance_ab, ZERO, TOLERANCE);
  EXPECT_NEAR(distance_bc, SMALL, TOLERANCE);
  EXPECT_NEAR(distance_ac, 2. * (std::sqrt(3.) - radius), TOLERANCE);
}

// Test that we can find the distance between two boxes in 3D when they
// collide face-to-face.
TEST(GJKAlgorithmTest, Test3DBoxesFaceFaceCollision) {
  // SETUP
  // A and B should intersect, B and C should barely not intersect.
  const Vec3 box_a_translation{Vec3::Zero()};
  const Vec3 box_a_extents{Vec3::Ones()};

  const Vec3 box_b_translation{(ONE - SMALL) * Vec3::UnitX()};
  const Vec3 box_b_extents{Vec3::Ones()};

  const Vec3 box_c_translation{(ONE + SMALL) * Vec3::UnitX()};
  const Vec3 box_c_extents{Vec3::Ones()};

  const auto box_a = OrientedBox{SE3{box_a_translation}, box_a_extents};
  const auto box_b = OrientedBox{SE3{box_b_translation}, box_b_extents};
  const auto box_c = OrientedBox{SE3{box_c_translation}, box_c_extents};

  const auto support_a{box_support(box_a)};
  const auto support_b{box_support(box_b)};
  const auto support_c{box_support(box_c)};

  // ACTION
  const double distance_ab = *gjk_algorithm(support_a, support_b);
  const double distance_ac = *gjk_algorithm(support_a, support_c);

  // VERIFICATION
  EXPECT_NEAR(distance_ab, ZERO, TOLERANCE);
  EXPECT_NEAR(distance_ac, SMALL, TOLERANCE);
}

// Test that we can find the distance between two boxes in 3D with they collide
// edge-to-edge without any vertices of box A residing in box B. To accomplish
// this, box A is set up with a 45 degree spin about the z axis so the x and
// axes intersect the vertical edges of the box. box B and C are set up along
// the x axis with a 45 degree spin about the y axis.
TEST(GJKAlgorithmTest, Test3DBoxesEdgeEdgeCollision) {
  // SETUP
  // A and B should intersect, B and C should barely not intersect.
  const SO3 box_a_rotation{SO3::exp(M_PI_4 * Vec3::UnitZ())};
  const Vec3 box_a_translation{Vec3::Zero()};
  const Vec3 box_a_extents{Vec3::Ones()};

  const SO3 box_b_rotation{SO3::exp(M_PI_4 * Vec3::UnitY())};
  const Vec3 box_b_translation{(M_SQRT2 - SMALL) * Vec3::UnitX()};
  const Vec3 box_b_extents{Vec3::Ones()};

  const SO3 box_c_rotation{SO3::exp(M_PI_4 * Vec3::UnitY())};
  const Vec3 box_c_translation{(M_SQRT2 + SMALL) * Vec3::UnitX()};
  const Vec3 box_c_extents{Vec3::Ones()};

  const auto box_a =
      OrientedBox{SE3{box_a_rotation, box_a_translation}, box_a_extents};
  const auto box_b =
      OrientedBox{SE3{box_b_rotation, box_b_translation}, box_b_extents};
  const auto box_c =
      OrientedBox{SE3{box_c_rotation, box_c_translation}, box_c_extents};

  const auto support_a{box_support(box_a)};
  const auto support_b{box_support(box_b)};
  const auto support_c{box_support(box_c)};

  // ACTION
  const double distance_ab = *gjk_algorithm(support_a, support_b);
  const double distance_ac = *gjk_algorithm(support_a, support_c);

  // VERIFICATION
  EXPECT_NEAR(distance_ab, ZERO, TOLERANCE);
  EXPECT_NEAR(distance_ac, SMALL, TOLERANCE);
}

// Test a bunch of random box pairs to make sure we terminate in a reasonable
// number of steps. In this case, the boxes are not colliding.
TEST(GJKAlgorithmTest, Test3DBoxesRobustnessNoCollide) {
  constexpr std::size_t SEED = 892U;
  std::mt19937 rng{SEED};

  const Vec3 box_a_translation{-5.0, -4.0, -3.0};
  const Vec3 box_b_translation{6.0, 7.0, 8.0};

  constexpr int NUM_TESTS = 10000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const auto box_a = random_box(box_a_translation, rng);
    const auto box_b = random_box(box_b_translation, rng);
    const auto support_a{box_support(box_a)};
    const auto support_b{box_support(box_b)};

    const double distance = *gjk_algorithm(support_a, support_b);

    const double center_to_center_distance =
        (box_b_translation - box_a_translation).norm();

    EXPECT_LE(
        distance,
        center_to_center_distance -
            0.5 * (box_a.extents().minCoeff() + box_b.extents().minCoeff()));

    EXPECT_GE(
        distance,
        center_to_center_distance -
            0.5 * (box_a.extents().norm() + box_b.extents().norm()));
  }
}

// Test a bunch of random box pairs to make sure we terminate in a reasonable
// number of steps.In this case, the boxes are colliding.
TEST(GJKAlgorithmTest, Test3DBoxesRobustnessCollide) {
  constexpr std::size_t SEED = 892U;
  std::mt19937 rng{SEED};

  const Vec3 zero_translation{Vec3::Zero()};

  constexpr int NUM_TESTS = 10000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const auto box_a = random_box(zero_translation, rng);
    const auto box_b = random_box(zero_translation, rng);
    const auto support_a{box_support(box_a)};
    const auto support_b{box_support(box_b)};

    const double distance = *gjk_algorithm(support_a, support_b);

    EXPECT_NEAR(distance, ZERO, TOLERANCE);
  }
}

// Test that we fail when no convergence occurs.
TEST(GJKAlgorithmTest, NoConvergence3D) {
  // SETUP
  const auto support_a{
      sphere_support<3>(Eigen::Matrix<double, 3, 1>::Ones(), ONE)};
  const auto support_b{
      sphere_support<3>(-Eigen::Matrix<double, 3, 1>::Ones(), ONE)};

  constexpr int MAX_ITERATIONS = 1U;

  // ACTION / VERIFICATION
  EXPECT_FALSE(gjk_algorithm(support_a, support_b, MAX_ITERATIONS).has_value());
}

}  // namespace resim::geometry
