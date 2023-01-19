#include "resim_core/planning/ilqr.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <random>
#include <utility>

#include "resim_core/testing/random_matrix.hh"

namespace resim::planning {

namespace {

// A test state to be used for testing iLQR
template <int DIM_t>
struct TestState {
  static constexpr int DIM = DIM_t;
  using Vec = Eigen::Matrix<double, DIM, 1>;
  Vec x{Eigen::Matrix<double, DIM, 1>::Zero()};
};

template <int DIM_t>
TestState<DIM_t> operator+(
    const TestState<DIM_t> &x,
    const typename TestState<DIM_t>::Vec &dx) {
  return {.x = x.x + dx};
}

template <int DIM_t>
typename TestState<DIM_t>::Vec operator-(
    const TestState<DIM_t> &x,
    const TestState<DIM_t> &y) {
  return x.x - y.x;
}

// A test control to be used for testing iLQR
template <int DIM_t>
struct TestControl {
  static constexpr int DIM = DIM_t;
  using Vec = Eigen::Matrix<double, DIM, 1>;
  Vec u{Eigen::Matrix<double, DIM, 1>::Zero()};
};

template <int DIM_t>
TestControl<DIM_t> operator+(
    const TestControl<DIM_t> &u,
    const typename TestControl<DIM_t>::Vec &du) {
  return {.u = u.u + du};
}

template <int DIM_t>
typename TestControl<DIM_t>::Vec operator-(
    const TestControl<DIM_t> &u,
    const TestControl<DIM_t> &v) {
  return u.u - v.u;
}

// Make random linear test dynamics for the test state and control defined
// above.
template <int STATE_DIM, int CONTROL_DIM, class Rng>
DiscreteDynamics<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>
make_random_linear_dynamics(Rng &&rng) {
  using MatXX = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  using MatXU = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;
  using VecX = Eigen::Matrix<double, STATE_DIM, 1>;

  const MatXX A{testing::random_matrix<MatXX>(rng)};
  const MatXU B{testing::random_matrix<MatXU>(rng)};
  const VecX c{testing::random_vector<VecX>(rng)};
  return [A, B, c](
             const TestState<STATE_DIM> &state,
             const TestControl<CONTROL_DIM> &control,
             const NullableReference<
                 DynamicsDiffs<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>>
                 diffs) {
    if (diffs) {
      diffs->f_x = A;
      diffs->f_u = B;
    }
    return TestState<STATE_DIM>{.x = A * state.x + B * control.u + c};
  };
}

// Make a random quadratic cost for the test state and test control defined
// above. Note that the resulting cost is constructed to be non-negative at all
// points.
template <int STATE_DIM, int CONTROL_DIM, class Rng>
CostFunction<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>
make_random_quadratic_cost(Rng &&rng) {
  using VecX = Eigen::Matrix<double, STATE_DIM, 1>;
  using VecU = Eigen::Matrix<double, CONTROL_DIM, 1>;
  using MatXX = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  using MatUX = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;
  using MatUU = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;

  // Create an overall system matrix (combining state and control)
  // which we can guarantee to be PD by construction. The cost can be
  // expressed in terms of this matrix like so:
  //
  //         1 [ 1]^T [ 1  K_x^T  K_u^T ] [ 1]
  // cost =  - [dx]   [K_x K_xx   K_ux^T] [dx]
  //         2 [du]   [K_u K_ux   k_uu  ] [du]
  //
  constexpr int TOTAL_DIM = STATE_DIM + CONTROL_DIM + 1;
  using MatTT = Eigen::Matrix<double, TOTAL_DIM, TOTAL_DIM>;
  const MatTT precursor{testing::random_matrix<MatTT>(rng)};
  const MatTT sys_matrix{precursor * precursor.transpose()};

  const MatXX K_xx{sys_matrix.template block<STATE_DIM, STATE_DIM>(1, 1)};
  const MatUU K_uu{sys_matrix.template block<CONTROL_DIM, CONTROL_DIM>(
      STATE_DIM + 1,
      STATE_DIM + 1)};
  const MatUX K_ux{
      sys_matrix.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM + 1, 1)};

  const VecX K_x{sys_matrix.template block<STATE_DIM, 1>(1, 0)};
  const VecU K_u{sys_matrix.template block<CONTROL_DIM, 1>(STATE_DIM + 1, 0)};

  constexpr double HALF = 0.5;
  const double constant = HALF * sys_matrix(0, 0);

  return [HALF, constant, K_xx, K_uu, K_ux, K_x, K_u](
             const TestState<STATE_DIM> &state,
             const NullableReference<const TestControl<CONTROL_DIM>> control,
             const NullableReference<
                 CostDiffs<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>>
                 diffs) {
    double cost = constant;
    cost += HALF * state.x.transpose() * K_xx * state.x;
    cost += K_x.transpose() * state.x;

    if (diffs) {
      *diffs = {};
      diffs->cost_x = K_xx * state.x + K_x;
      diffs->cost_xx = K_xx;
    }

    if (control) {
      cost += HALF * control->u.transpose() * K_uu * control->u;
      cost += control->u.transpose() * K_ux * state.x;
      cost += K_u.transpose() * control->u;
      if (diffs) {
        diffs->cost_x += K_ux.transpose() * control->u;
        diffs->cost_u = K_uu * control->u + K_ux * state.x + K_u;
        diffs->cost_ux = K_ux;
        diffs->cost_uu = K_uu;
      }
    }
    return cost;
  };
}

// Make a random non-quadratic cost for the test state and test control defined
// above. Do this by taking a non-negative quadratic cost and passing it through
// a scalar function of f(x) = ax - exp(-x). We choose this function so that the
// resulting cost function will have some non-convex areas. For example, in one
// dimension, a quadratic cost might just be x^2 and then this function could
// output 0.125x^2 - exp(-x^2) which is not convex at x = 1.
template <int STATE_DIM, int CONTROL_DIM, class Rng>
CostFunction<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>
make_random_non_quadratic_cost(Rng &&rng) {
  auto base_cost = make_random_quadratic_cost<STATE_DIM, CONTROL_DIM>(rng);
  return [base_cost = std::move(base_cost)](
             const TestState<STATE_DIM> &state,
             const NullableReference<const TestControl<CONTROL_DIM>> control,
             const NullableReference<
                 CostDiffs<TestState<STATE_DIM>, TestControl<CONTROL_DIM>>>
                 diffs) {
    const double quadratic_cost = base_cost(state, control, diffs);
    constexpr double A = 0.1;
    const double cost = A * quadratic_cost - std::exp(-quadratic_cost);
    const double d_cost = A + std::exp(-quadratic_cost);
    const double d2_cost = -std::exp(-quadratic_cost);
    if (diffs) {
      // Update these in reverse order to make sure we don't overwrite things
      // we still need.
      diffs->cost_xx = d2_cost * (diffs->cost_x * diffs->cost_x.transpose()) +
                       d_cost * diffs->cost_xx;
      diffs->cost_ux = d2_cost * (diffs->cost_u * diffs->cost_x.transpose()) +
                       d_cost * diffs->cost_ux;
      diffs->cost_uu = d2_cost * (diffs->cost_u * diffs->cost_u.transpose()) +
                       d_cost * diffs->cost_uu;
      diffs->cost_x = d_cost * diffs->cost_x;
      diffs->cost_u = d_cost * diffs->cost_u;
    }
    return cost;
  };
}

// A simple struct template to hold dimensions so we can have a typed test try
// many dimensions.
template <int STATE_DIM, int CONTROL_DIM>
struct Dims {
  static constexpr int STATE = STATE_DIM;
  static constexpr int CONTROL = CONTROL_DIM;
};

// Setup the typed test suite
// NOLINTBEGIN(readability-magic-numbers)
using DimsToTest = ::testing::Types<
    Dims<2, 2>,
    Dims<3, 2>,
    Dims<4, 2>,
    Dims<3, 3>,
    Dims<5, 2>,
    Dims<4, 3>,
    Dims<6, 2>,
    Dims<5, 3>,
    Dims<4, 4>>;
// NOLINTEND(readability-magic-numbers)

// A simple helper to integrate the dynamics with the result controls and ensure
// they match what's in the states vector of the given ILQR result.
template <class State, class Control>
void expect_states_controls_consistent(
    const DiscreteDynamics<State, Control> &dynamics,
    const typename ILQR<State, Control>::Result &result) {
  const std::size_t num_steps = result.controls.size();
  ASSERT_EQ(result.states.size(), num_steps + 1U);
  for (std::size_t state_index = 0U; state_index < num_steps; ++state_index) {
    auto states = result.states;
    const auto no_diffs = null_reference<DynamicsDiffs<State, Control>>;
    states.at(state_index + 1U) = dynamics(
        states.at(state_index),
        result.controls.at(state_index),
        no_diffs);
    EXPECT_EQ(
        states.at(state_index + 1U).x,
        result.states.at(state_index + 1U).x);
  }
}

// A simple helper to check that the cost increases if we perturb any individual
// control in the control sequence.
template <class State, class Control>
void expect_controls_optimal(
    const DiscreteDynamics<State, Control> &dynamics,
    const CostFunction<State, Control> &cost_fn,
    const typename ILQR<State, Control>::Result &result) {
  const std::size_t num_steps = result.controls.size();
  ASSERT_EQ(result.states.size(), num_steps + 1U);

  for (std::size_t perturbed_index = 0U; perturbed_index < num_steps;
       ++perturbed_index) {
    for (int ii = 0; ii < Control::DIM; ++ii) {
      auto perturbed_controls = result.controls;
      constexpr double EPSILON = 1e-4;
      perturbed_controls.at(perturbed_index).u(ii) += EPSILON;
      auto perturbed_states = result.states;

      double cost = 0.;
      for (std::size_t state_index = 0U; state_index < num_steps;
           ++state_index) {
        {
          const auto no_diffs = null_reference<DynamicsDiffs<State, Control>>;
          perturbed_states.at(state_index + 1U) = dynamics(
              perturbed_states.at(state_index),
              perturbed_controls.at(state_index),
              no_diffs);
        }
        {
          const auto no_diffs = null_reference<CostDiffs<State, Control>>;
          cost += cost_fn(
              perturbed_states.at(state_index),
              NullableReference<const Control>{
                  perturbed_controls.at(state_index)},
              no_diffs);
        }
      }
      const auto no_diffs = null_reference<CostDiffs<State, Control>>;
      const auto no_controls = null_reference<const Control>;
      cost += cost_fn(perturbed_states.at(num_steps), no_controls, no_diffs);
      EXPECT_GT(cost, result.cost);
    }
  }
}

enum class CostType {
  QUADRATIC = 0,
  NON_QUADRATIC,
};

// Test ILQR once with randomly generated dynamics and non-quadratic cost.
template <int STATE_DIM, int CONTROL_DIM, CostType cost_type, class Rng>
void test_ilqr_once(Rng &&rng) {
  // SETUP
  constexpr std::size_t NUM_STEPS = 15U;
  const auto dynamics =
      make_random_linear_dynamics<STATE_DIM, CONTROL_DIM>(rng);
  const auto cost =
      cost_type == CostType::QUADRATIC
          ? make_random_quadratic_cost<STATE_DIM, CONTROL_DIM>(rng)
          : make_random_non_quadratic_cost<STATE_DIM, CONTROL_DIM>(rng);

  ILQR<TestState<STATE_DIM>, TestControl<CONTROL_DIM>> ilqr{
      NUM_STEPS,
      dynamics,
      cost};

  constexpr std::size_t MAX_ITERATIONS = 100;
  constexpr std::size_t MAX_BACKTRACK_ITERATIONS = 100;
  constexpr double TOLERANCE = 1e-8;

  std::vector<TestControl<CONTROL_DIM>> initial_controls{NUM_STEPS};
  const TestState<STATE_DIM> initial_state{
      .x = testing::random_vector<Eigen::Matrix<double, STATE_DIM, 1>>(rng)};

  // ACTION
  const auto result = ilqr.optimize_controls(
      initial_state,
      initial_controls,
      MAX_ITERATIONS,
      MAX_BACKTRACK_ITERATIONS,
      TOLERANCE);

  // VERIFICATION
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.states.size(), NUM_STEPS + 1U);
  EXPECT_EQ(result.controls.size(), NUM_STEPS);
  EXPECT_EQ(result.states.front().x, initial_state.x);

  // Check that states are consistent with controls
  expect_states_controls_consistent(dynamics, result);

  // Check that perturbations to the controls increases the cost
  expect_controls_optimal(dynamics, cost, result);
}

// Test that we don't converge when we pass too few iterations to ILQR and that
// we report this correctly.
template <int STATE_DIM, int CONTROL_DIM, class Rng>
void test_ilqr_too_few_iterations(Rng &&rng) {
  // SETUP
  constexpr std::size_t NUM_STEPS = 15U;
  const auto dynamics =
      make_random_linear_dynamics<STATE_DIM, CONTROL_DIM>(rng);
  const auto cost = make_random_quadratic_cost<STATE_DIM, CONTROL_DIM>(rng);

  ILQR<TestState<STATE_DIM>, TestControl<CONTROL_DIM>> ilqr{
      NUM_STEPS,
      dynamics,
      cost};

  constexpr std::size_t MAX_ITERATIONS = 1;
  constexpr std::size_t MAX_BACKTRACK_ITERATIONS = 0;
  constexpr double TOLERANCE = 1e-8;

  std::vector<TestControl<CONTROL_DIM>> initial_controls{NUM_STEPS};
  const TestState<STATE_DIM> initial_state{
      .x = testing::random_vector<Eigen::Matrix<double, STATE_DIM, 1>>(rng)};

  // ACTION
  const auto result = ilqr.optimize_controls(
      initial_state,
      initial_controls,
      MAX_ITERATIONS,
      MAX_BACKTRACK_ITERATIONS,
      TOLERANCE);

  // VERIFICATION
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.states.size(), NUM_STEPS + 1U);
  EXPECT_EQ(result.controls.size(), NUM_STEPS);
  EXPECT_EQ(result.states.front().x, initial_state.x);
}

}  // namespace

template <typename T>
class ILQRTest : public ::testing::Test {
 protected:
  static constexpr uint64_t SEED = 4968U;
  std::mt19937 rng_{SEED};
};

TYPED_TEST_SUITE(ILQRTest, DimsToTest);

// Test that iLQR converges to the optimum over a variety of different dynamics,
// cost functions, and initial conditions.
TYPED_TEST(ILQRTest, TestILQR) {
  constexpr int NUM_TESTS = 5;
  for (int test_index = 0; test_index < NUM_TESTS; ++test_index) {
    test_ilqr_once<TypeParam::STATE, TypeParam::CONTROL, CostType::QUADRATIC>(
        this->rng_);
    test_ilqr_once<
        TypeParam::STATE,
        TypeParam::CONTROL,
        CostType::NON_QUADRATIC>(this->rng_);
  }
}

TYPED_TEST(ILQRTest, TestZeroSteps) {
  test_ilqr_too_few_iterations<TypeParam::STATE, TypeParam::CONTROL>(
      this->rng_);
}

}  // namespace resim::planning
