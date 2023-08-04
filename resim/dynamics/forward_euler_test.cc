// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/forward_euler.hh"

#include <gtest/gtest.h>

#include <chrono>

#include "resim/dynamics/controller.hh"
#include "resim/dynamics/dynamics.hh"
#include "resim/dynamics/testing/oscillator.hh"
#include "resim/time/sample_interval.hh"

namespace resim::dynamics {

// Test that Forward Euler correctly integrates the oscillator system.
TEST(ForwardEulerTest, TestForwardEuler) {
  // SETUP
  constexpr double STIFFNESS = 2.0;
  constexpr double INITIAL_VELOCITY = 1.5;
  const double omega_radps = std::sqrt(STIFFNESS);

  const std::unique_ptr<Integrator<testing::OscillatorState>> integrator =
      std::make_unique<ForwardEuler<testing::OscillatorState>>();
  const std::unique_ptr<
      Dynamics<testing::OscillatorState, testing::OSCILLATOR_CONTROL_DIM>>
      dynamics = std::make_unique<testing::OscillatorDynamics>();

  const std::unique_ptr<
      Controller<testing::OscillatorState, testing::OSCILLATOR_CONTROL_DIM>>
      controller = std::make_unique<testing::OscillatorController>(STIFFNESS);

  testing::OscillatorState state{
      .displacement = 0.,
      .velocity = INITIAL_VELOCITY,
  };

  constexpr time::Duration MAX_DT{std::chrono::microseconds(10)};
  constexpr time::Timestamp START_TIME;
  const time::Timestamp end_time{START_TIME + std::chrono::seconds(1)};

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

        // VERIFICATION
        // Analytical solution:
        const double expected_displacement =
            (INITIAL_VELOCITY / omega_radps) *
            std::sin(omega_radps * time::as_seconds(t - START_TIME));

        constexpr double TOLERANCE = 5e-5;
        EXPECT_NEAR(state.displacement, expected_displacement, TOLERANCE);

        last_time = t;
      });
}

}  // namespace resim::dynamics
