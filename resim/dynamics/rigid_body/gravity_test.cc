#include "resim/dynamics/rigid_body/gravity.hh"

#include <gtest/gtest.h>

#include <memory>
#include <random>

#include "au/au.hh"
#include "au/units/newtons.hh"
#include "resim/dynamics/constants.hh"
#include "resim/dynamics/controller.hh"
#include "resim/dynamics/rigid_body/state.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::dynamics::rigid_body {

using transforms::SE3;
using TangentVector = transforms::SE3::TangentVector;

// Check that gravity is as expected in a number of poses
TEST(GravityTest, TestGravity) {
  // SETUP
  constexpr unsigned SEED = 42U;
  std::mt19937 rng{SEED};

  constexpr auto MASS = au::kilo(au::grams)(22.0);

  const std::unique_ptr<Controller<State, SE3::DOF>> gravity_force{
      std::make_unique<GravityForce>(MASS)};

  const auto test_poses = transforms::make_test_group_elements<SE3>();
  for (const auto &pose : test_poses) {
    // ACTION
    State state{
        .reference_from_body = pose,
        .d_reference_from_body = testing::random_vector<TangentVector>(rng),
    };
    constexpr time::Timestamp TIME;
    const TangentVector force{
        (*gravity_force)(state, TIME, null_reference<GravityForce::Jacobian>)};

    // VERIFICATION
    EXPECT_TRUE(SE3::tangent_vector_rotation_part(force).isZero());
    EXPECT_TRUE((pose.rotation() * SE3::tangent_vector_translation_part(force))
                    .isApprox(-(MASS * GRAVITY_ACCELERATION)
                                   .in(au::newtons)*Eigen::Vector3d::UnitZ()));
  }
}

}  // namespace resim::dynamics::rigid_body
