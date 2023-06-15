#include "resim/dynamics/rigid_body/inertia.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/testing/random_matrix.hh"
#include "resim/transforms/se3.hh"

namespace resim::dynamics::rigid_body {
using transforms::SE3;
using TangentVector = SE3::TangentVector;

// Test that we can construct an inertia matrix from mass and moments.
TEST(InertiaTest, TestInertiaFromMassAndMoments) {
  // SETUP
  constexpr double MASS = 32.;
  const Eigen::Vector3d moments_of_inertia{5., 6., 4.};

  // ACTION
  const Inertia inertia{
      inertia_from_mass_and_moments_of_inertia(MASS, moments_of_inertia)};

  // VERIFICATION
  // Technically, we don't care about the representation of the inertia as long
  // as it behaves correctly. We do this by making sure we can correctly compute
  // kinetic energy with it.
  constexpr unsigned SEED = 43U;
  std::mt19937 rng{SEED};
  constexpr int NUM_TRIES = 100;
  for (int ii = 0; ii < NUM_TRIES; ++ii) {
    const TangentVector V{testing::random_vector<TangentVector>(rng)};

    const double kinetic_energy = V.transpose() * inertia * V;

    const Eigen::Vector3d w{SE3::tangent_vector_rotation_part(V)};
    const Eigen::Vector3d v{SE3::tangent_vector_translation_part(V)};
    const double expected_kinetic_energy =
        w.transpose() * moments_of_inertia.asDiagonal() * w +
        v.squaredNorm() * MASS;

    constexpr double TOLERANCE = 1e-10;
    EXPECT_NEAR(kinetic_energy, expected_kinetic_energy, TOLERANCE);
  }
}

}  // namespace resim::dynamics::rigid_body
