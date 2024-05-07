// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/dynamics.hh"

#include <gtest/gtest.h>

#include "resim/planning/drone/control.hh"
#include "resim/planning/drone/state.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::planning::drone {

TEST(PlannerTest, TestDynamics) {
  // SETUP
  State state{State() + State::Vec::Random()};
  Control control{Control() + Control::Vec::Random()};

  constexpr double DT = 0.1;
  constexpr double G = 9.81;
  const Dynamics dynamics(DT, G);

  // ACTION
  DynamicsDiffs<State, Control> diffs;
  State out{dynamics(state, control, NullableReference{diffs})};
  constexpr double EPSILON = 1e-6;
  constexpr double TOLERANCE = 1e-7;

  // VERIFICATION
  // Verify the differentials using finite differences:
  {
    DynamicsDiffs<State, Control>::MatXX jac;
    for (int ii = 0; ii < State::DIM; ++ii) {
      State::Vec delta{EPSILON * State::Vec::Unit(ii)};

      State state_perturbed{state + delta};
      State out_perturbed{dynamics(state_perturbed, control, null_reference)};

      jac.col(ii) = (out_perturbed - out) / EPSILON;
    }
    EXPECT_TRUE(math::is_approx(jac, diffs.f_x, TOLERANCE));
  }
  {
    DynamicsDiffs<State, Control>::MatXU jac;
    for (int ii = 0; ii < Control::DIM; ++ii) {
      Control::Vec delta{EPSILON * Control::Vec::Unit(ii)};

      Control control_perturbed{control + delta};
      State out_perturbed{dynamics(state, control_perturbed, null_reference)};

      jac.col(ii) = (out_perturbed - out) / EPSILON;
    }
    EXPECT_TRUE(math::is_approx(jac, diffs.f_u, TOLERANCE));
  }
}

}  // namespace resim::planning::drone
