// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/dynamics.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/dynamics/controller.hh"
#include "resim/dynamics/forward_euler.hh"
#include "resim/dynamics/rigid_body/inertia.hh"
#include "resim/dynamics/rigid_body/state.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/sample_interval.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::dynamics::rigid_body {

namespace {
using transforms::SE3;
using TangentVector = SE3::TangentVector;

constexpr int CONTROL_DIM = SE3::DOF;

//
// This controller is defined for test purposes to generate a constant
// generalized force in reference coordinates and then return it in body
// coordinates. Since the force is constant in reference coordinates, it will be
// changing in body coordinates as the body moves around.
class ConstantController : public Controller<State, CONTROL_DIM> {
 public:
  explicit ConstantController(TangentVector force) : force_{std::move(force)} {}

  TangentVector operator()(
      const State &state,
      const time::Timestamp time,
      NullableReference<Jacobian> jacobian) const override {
    REASSERT(not jacobian.has_value(), "Jacobian not supported!");
    // The force is in the reference frame, so convert it to the body
    // frame. Since it is in the cotangent space, we use equation (13) from
    // https://drive.google.com/file/d/1CBpHs88MsTx1971AWM3SzPOsAeba1VD1/view
    return state.reference_from_body.adjoint().transpose() * force_;
  }

  const TangentVector &force() const { return force_; }

 private:
  TangentVector force_;
};

// This function computes the momentum in the reference frame.
TangentVector compute_momentum_in_reference_frame(
    const State &state,
    const Inertia &inertia) {
  // Since the inertia and velocity are in body coordinates and we want the
  // momentum in reference coordinates, we have to transform the body frame
  // momentum. Since generalized momentum is in the cotangent space at the
  // current point, we use the same transformation rule used above for force.
  // Namely, since momentum is in the cotangent space, we use equation (13) from
  // https://drive.google.com/file/d/1CBpHs88MsTx1971AWM3SzPOsAeba1VD1/view
  return state.reference_from_body.inverse().adjoint().transpose() * inertia *
         state.d_reference_from_body;
}

}  // namespace

void test_rigid_body_dynamics_once(
    const Inertia &inertia,
    const TangentVector &force_in_reference_frame,
    const TangentVector &initial_d_reference_from_body) {
  const std::unique_ptr<Controller<State, CONTROL_DIM>> controller =
      std::make_unique<ConstantController>(force_in_reference_frame);

  std::unique_ptr<dynamics::Dynamics<State, CONTROL_DIM>> dynamics{
      std::make_unique<Dynamics>(inertia)};

  State state;
  state.d_reference_from_body = initial_d_reference_from_body;
  const TangentVector initial_momentum{
      compute_momentum_in_reference_frame(state, inertia)};

  constexpr time::Duration MAX_DT{std::chrono::microseconds(100)};
  constexpr time::Timestamp START_TIME;
  const time::Timestamp end_time{START_TIME + std::chrono::seconds(1)};

  std::unique_ptr<Integrator<State>> integrator =
      std::make_unique<ForwardEuler<State>>();

  // ACTION
  // Integrate forward over a second using Forward Euler
  time::Timestamp last_time = START_TIME;
  time::sample_interval(
      START_TIME,
      end_time,
      MAX_DT,
      [&](const time::Timestamp t) {
        // Do nothing on the first step
        if (t == last_time) {
          return;
        }
        const time::Duration dt{t - last_time};

        state = (*integrator)(dt, state, t, *dynamics, *controller);
        last_time = t;
      });

  // VERIFICATION
  const TangentVector final_momentum{
      compute_momentum_in_reference_frame(state, inertia)};

  const TangentVector momentum_change{final_momentum - initial_momentum};
  const TangentVector impulse{
      force_in_reference_frame * time::as_seconds(end_time - START_TIME)};

  constexpr double TOLERANCE = 2e-3;
  EXPECT_NEAR((momentum_change - impulse).norm(), 0.0, TOLERANCE);
}

// This check confirms that our dynamics are correct by integrating them and
// verifying that the momentum change matches the impulse.
TEST(RigidBodyDynamicsTest, TestRigidBodyDynamics) {
  // SETUP
  constexpr unsigned SEED = 8934U;
  std::mt19937 rng{SEED};
  constexpr double MASS = 2.0;
  const Eigen::Vector3d moments_of_inertia{1.1, 1.2, 1.3};
  const Inertia inertia{
      inertia_from_mass_and_moments_of_inertia(MASS, moments_of_inertia)};

  const auto test_forces{transforms::make_test_algebra_elements<SE3>()};
  const auto test_initial_d_references_from_body{
      transforms::make_test_algebra_elements<SE3>()};

  for (TangentVector force : test_forces) {
    for (TangentVector initial_d_reference_from_body :
         test_initial_d_references_from_body) {
      // Forward Euler is not stable for these dynamics if the forces or
      // velocities are very large, but we still want the other edge cases.
      const auto clamp_vector = [](const TangentVector &a,
                                   const double max_norm) -> TangentVector {
        return std::min(max_norm, a.norm()) * a.normalized();
      };
      constexpr double MAX_FORCE = 3.0;
      constexpr double MAX_SPEED = 3.0;
      force = clamp_vector(force, MAX_FORCE);
      initial_d_reference_from_body =
          clamp_vector(initial_d_reference_from_body, MAX_SPEED);

      test_rigid_body_dynamics_once(
          inertia,
          force,
          initial_d_reference_from_body);
    }
  }
}

// Test the Jacobian by testing that small perturbations to the inputs behave
// the way the Jacobian predicts.
TEST(RigidBodyDynamicsTest, TestJacobian) {
  // SETUP
  constexpr double MASS = 2.0;
  const Eigen::Vector3d moments_of_inertia{1.1, 1.2, 1.3};
  const Inertia inertia{
      inertia_from_mass_and_moments_of_inertia(MASS, moments_of_inertia)};
  Dynamics dynamics{inertia};
  constexpr time::Timestamp START_TIME;

  constexpr int NUM_TESTS = 100;
  constexpr size_t SEED = 87U;
  std::mt19937 rng{SEED};
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const State state{State() + testing::random_vector<State::Delta>(rng)};
    using Control = TangentVector;
    const Control force{testing::random_vector<Control>(rng)};
    Dynamics::Diffs diffs;

    constexpr double EPSILON = 1e-7;
    const State::Delta perturbation{
        EPSILON * testing::random_vector<State::Delta>(rng)};
    const State state_perturbed{state + perturbation};

    const Control force_perturbation{
        EPSILON * testing::random_vector<Control>(rng)};
    const Control force_perturbed{force + force_perturbation};

    // ACTION
    const State::Delta output{
        dynamics(state, force, START_TIME, NullableReference{diffs})};

    // VERIFICATION
    const State::Delta output_perturbed{
        dynamics(state_perturbed, force_perturbed, START_TIME, null_reference)};

    // Using Taylor series
    const State::Delta expected_output_perturbed{
        output + diffs.f_x * perturbation + diffs.f_u * force_perturbation};

    constexpr double TOLERANCE = 1e-8;
    EXPECT_TRUE(((expected_output_perturbed - output_perturbed).array() /
                 output.array())
                    .matrix()
                    .isZero(TOLERANCE));
  }
}

// Test to make sure we cover the instantations of NullableReference for the
// controller jacobian type.
TEST(CoverageTest, ControllerReferenceCoverage) {
  {
    ConstantController::Jacobian j;
    EXPECT_EQ(&*NullableReference{j}, &j);
  }
  {
    Integrator<State>::MatXX mat;
    EXPECT_EQ(&*NullableReference{mat}, &mat);
  }
}

}  // namespace resim::dynamics::rigid_body
