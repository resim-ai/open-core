

#include "resim/planning/cost_function_registry.hh"

#include <gtest/gtest.h>

#include "resim/planning/cost_function.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::planning {

namespace {

// Test states and control types for cost function registry
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

}  // namespace

TEST(CostFunctionRegistryTest, TestRegistry) {
  // SETUP
  constexpr int DIM = 3;
  using State = TestState<DIM>;
  using Control = TestControl<DIM>;
  using Diffs = CostDiffs<State, Control>;
  using Registry = CostFunctionRegistry<State, Control>;
  constexpr size_t SEED = 32U;
  std::mt19937 rng{SEED};

  auto get_cost = [&rng](bool has_control_cost) {
    constexpr size_t BIG_DIM = State::DIM + Control::DIM;
    using BigMat = Eigen::Matrix<double, BIG_DIM, BIG_DIM>;
    using BigVec = Eigen::Matrix<double, BIG_DIM, 1>;
    BigMat curvature{testing::random_matrix<BigMat>(rng)};
    curvature = curvature * curvature;

    const BigVec bias{testing::random_matrix<BigVec>(rng)};

    return [curvature, bias, has_control_cost](
               const State &x,
               NullableReference<const Control> u,
               NullableReference<CostDiffs<State, Control>> diffs) -> double {
      Eigen::Block<const BigMat, State::DIM, State::DIM> Kxx{
          curvature.block<State::DIM, State::DIM>(0, 0)};
      Eigen::Block<const BigMat, State::DIM, State::DIM> Kux{
          curvature.block<Control::DIM, State::DIM>(State::DIM, 0)};
      Eigen::Block<const BigMat, State::DIM, State::DIM> Kuu{
          curvature.block<Control::DIM, Control::DIM>(State::DIM, State::DIM)};

      constexpr double HALF = 0.5;
      double cost = HALF * x.x.transpose() * Kxx * x.x;

      if (diffs.has_value()) {
        diffs->cost_x += Kxx * x.x;
        diffs->cost_xx += Kxx;
      }

      if (has_control_cost and u.has_value()) {
        cost +=
            (u->u.transpose() * Kux * x.x + u->u.transpose() * Kuu * u->u).x();
        if (diffs.has_value()) {
          diffs->cost_u += Kuu * u->u;
          diffs->cost_ux += Kux;
          diffs->cost_uu += Kuu;
        }
      }
      return cost;
    };
  };

  Registry registry;
  constexpr bool HAS_CONTROL_COSTS = true;
  constexpr bool NO_CONTROL_COSTS = false;
  registry["cost_1"] = get_cost(HAS_CONTROL_COSTS);
  registry["cost_2"] = get_cost(NO_CONTROL_COSTS);
  registry["cost_3"] = get_cost(HAS_CONTROL_COSTS);

  const State x{State() + testing::random_vector<State::Vec>(rng)};
  const Control u{Control() + testing::random_vector<Control::Vec>(rng)};
  Diffs diffs;

  // ACTION
  double cost = registry(x, NullableReference{u}, NullableReference{diffs});

  // VERIFICATION
  for (const std::string &key : {"cost_1", "cost_2", "cost_3"}) {
    const auto &fn = registry[key];
    Diffs term_diffs;
    cost -= fn(x, NullableReference{u}, NullableReference{term_diffs});

    diffs.cost_x -= term_diffs.cost_x;
    diffs.cost_u -= term_diffs.cost_u;
    diffs.cost_xx -= term_diffs.cost_xx;
    diffs.cost_ux -= term_diffs.cost_ux;
    diffs.cost_uu -= term_diffs.cost_uu;
  }
  EXPECT_EQ(cost, 0.0);
  EXPECT_TRUE(diffs.cost_x.isZero());
  EXPECT_TRUE(diffs.cost_u.isZero());
  EXPECT_TRUE(diffs.cost_xx.isZero());
  EXPECT_TRUE(diffs.cost_ux.isZero());
  EXPECT_TRUE(diffs.cost_uu.isZero());
}

}  // namespace resim::planning
