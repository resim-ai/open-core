// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/integrator.hh"

#include <gtest/gtest.h>

#include <chrono>

#include "resim/dynamics/controller.hh"
#include "resim/dynamics/dynamics.hh"
#include "resim/dynamics/testing/oscillator.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::dynamics {

// This is a zero-th order test integrator defined only to help us test that the
// Integrator interface correctly forwards Dynamics + Controller to the full
// dynamics overload. All it does is return the input state while checking that
// the time and timestep match the expected values that this integrator is
// constructed with. It also uses finite differences to verify that the
// Jacobians are correctly computed via the chain rule.
class TestIntegrator : public Integrator<testing::OscillatorState> {
 public:
  TestIntegrator(time::Duration expected_dt, time::Timestamp expected_time)
      : expected_dt_{expected_dt},
        expected_time_{expected_time} {}

  State operator()(
      const time::Duration dt,
      const State &state,
      const time::Timestamp time,
      const CompleteDynamics &complete_dynamics) override {
    EXPECT_EQ(dt, expected_dt_);
    EXPECT_EQ(time, expected_time_);

    MatXX jacobian{MatXX::Zero()};
    const Delta delta{
        complete_dynamics(state, time, NullableReference{jacobian})};

    // Check the Jacobian by finite differences:
    for (int ii = 0; ii < delta.size(); ++ii) {
      constexpr double EPSILON = 1e-8;
      const Delta perturbation{EPSILON * Delta::Unit(ii)};
      const State perturbed_state{state + perturbation};
      const Delta perturbed_delta{
          complete_dynamics(perturbed_state, time, null_reference<MatXX>)};
      EXPECT_TRUE(
          (perturbed_delta - delta).isApprox(jacobian * perturbation, EPSILON));
    }
    return state;
  }

 private:
  time::Duration expected_dt_;
  time::Timestamp expected_time_;
};

// This test checks that the integrator interface correctly forwards the
// Dynamics + Controller overload to the complete dynamics overload, which
// implementations are expected to override.
TEST(IntegratorTest, TestIntegrator) {
  // SETUP
  constexpr double STIFFNESS = 2.0;
  constexpr double INITIAL_VELOCITY = 1.5;
  constexpr time::Timestamp TIME;
  constexpr time::Duration DT{std::chrono::microseconds(10)};

  const std::unique_ptr<
      Dynamics<testing::OscillatorState, testing::OSCILLATOR_CONTROL_DIM>>
      dynamics = std::make_unique<testing::OscillatorDynamics>();

  const std::unique_ptr<
      Controller<testing::OscillatorState, testing::OSCILLATOR_CONTROL_DIM>>
      controller = std::make_unique<testing::OscillatorController>(STIFFNESS);

  const std::unique_ptr<Integrator<testing::OscillatorState>> integrator{
      std::make_unique<TestIntegrator>(DT, TIME)};

  testing::OscillatorState state{
      .displacement = 0.,
      .velocity = INITIAL_VELOCITY,
  };

  // ACTION / VERIFICATION
  (*integrator)(DT, state, TIME, *dynamics, *controller);
}

}  // namespace resim::dynamics
