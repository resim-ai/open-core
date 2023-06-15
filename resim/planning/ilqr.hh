#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <concepts>
#include <utility>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/planning/cost_function.hh"
#include "resim/planning/dynamics.hh"
#include "resim/utils/double_buffer.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::planning {

// Define some concepts needed to enforce behaviors of State and Controls used
// in ILQR.

template <typename T>
concept StateType = requires(T x) {
  // Our states must specify their dimension
  T::DIM;
  // Must be able to find a difference between two states
  { x - x } -> std::same_as<Eigen::Matrix<double, T::DIM, 1U>>;
  // Must be possible to add pertubations/updates to our states
  { x + Eigen::Matrix<double, T::DIM, 1U>() } -> std::same_as<T>;
};

template <typename T>
concept ControlType = requires(T u) {
  // Our controls must specify their dimension
  T::DIM;
  // Must be possible to add pertubations/updates to our controls
  { u + Eigen::Matrix<double, T::DIM, 1U>() } -> std::same_as<T>;
};

//
// This class contains an implementation of the iterative linear quadratic
// regular algorithm (iLQR). In particular, given discrete system dynamics and a
// cost function, this algorithm can be used to find an optimal sequence of
// controls for that system under that cost. The implementation herein is based
// on the following paper:
//
// https://drive.google.com/file/d/1v6HWMBH6bbRyqtQ6wul92lE4URipa2BT/view
//
template <StateType State, ControlType Control>
class ILQR {
 public:
  // The regularization scheme we're applying requires this. Otherwise, the
  // expression f_u^T (mu I) f_u will be singular and adding it to Q_uu will not
  // make it positive definite as mu -> Inf.
  static_assert(
      State::DIM >= Control::DIM,
      "State dimension must be no smaller than control dimension!");
  using MatXX = Eigen::Matrix<double, State::DIM, State::DIM>;
  using MatXU = Eigen::Matrix<double, State::DIM, Control::DIM>;
  using MatUX = Eigen::Matrix<double, Control::DIM, State::DIM>;
  using MatUU = Eigen::Matrix<double, Control::DIM, Control::DIM>;
  using VecX = Eigen::Matrix<double, State::DIM, 1>;
  using VecU = Eigen::Matrix<double, Control::DIM, 1>;

  // The result of the iLQR algorithm.
  struct Result {
    bool converged = false;

    // The number of iterations taken by the algorithm not including line search
    // iterations.
    std::size_t num_iterations = 0U;

    // The optimal cost
    double cost = 0.0;

    // The optimal states and controls
    std::vector<State> states;
    std::vector<Control> controls;
  };

  // Constructor
  // @param[in] num_steps - The number of steps we expect to run iLQR on. We use
  //                        this to pre-allocate intermediate vectors since we
  //                        expect to use this object for many optimizations.
  // @param[in] dynamics - The dynamics that we expect to optimize controls on.
  // @param[in] cost - The cost function we want to optimize under.
  ILQR(
      std::size_t num_steps,
      DiscreteDynamics<State, Control> dynamics,
      CostFunction<State, Control> cost);

  // Find the optimal set of controls starting at the given initial state.
  // @param[in] initial_state - The first state to begin optimizing from.
  // @param[in] initial_controls - A guess sequence of controls to begin
  //                               optimizing from.
  // @param[in] max_iterations - The maximum number of primary (non-backtrack)
  //                             iterations to take.
  // @param[in] max_backtrack_iterations - The maximum number of backtrack
  //                                       iterations to take per primary
  //                                       iteration.
  // @param[in] tolerance - The absolute difference between subsequent iteration
  //                        costs used to determine convergence.
  Result optimize_controls(
      const State &initial_state,
      const std::vector<Control> &initial_controls,
      std::size_t max_iterations = 10,
      std::size_t max_backtrack_iterations = 10,
      double tolerance = 1e-6);

 private:
  // The quadratic regularization parameters as described in the paper linked
  // above.
  struct RegularizationParameters {
    static constexpr double DEFAULT_MU_MIN = 1e-6;
    static constexpr double DEFAULT_DELTA_0 = 2.;
    double mu = 1.0;
    double delta = 1.;
    double mu_min = DEFAULT_MU_MIN;
    double delta_0 = DEFAULT_DELTA_0;
  };

  // Reset the states and controls double buffers, the feedforward and feedback
  // terms (to zero), and the regularization parameters.
  // @param[in] initial_state - An initial state to put in the start of both
  //                            state buffers.
  // @param[in] initial_controls - The initial controls to put in the current
  //                               controls buffer (but not the next one).
  void reset(
      const State &initial_state,
      const std::vector<Control> &initial_controls);

  // Take a forward pass. Compute new controls into the next buffer based on the
  // feedforward and feedback terms and the current controls buffer. Use the new
  // controls to integrate the next initial state into the next state buffer.
  // @param[in] linesearch_factor - The linesearch factor to use when computing
  //                                new controls (i.e. alpha from the reference
  //                                paper).
  double forward_pass(double linesearch_factor);

  // Compute the dynamics and cost derivatives for each step.
  void compute_derivatives();

  // Compute the backward pass, finding the first and second state derivatives
  // of the value function recursively for each step, and using these to compute
  // the feedback and feedforward terms. Do this by using the following member
  // function and adjusting the regularization as necessary.
  void backward_pass();

  // Try one backward pass as described above. This can fail if we run into
  // numerical issues (i.e. a non-positive-definite Q_uu matrix) that
  // necessitate increased regularization.
  bool try_backward_pass();

  // Update the regularization parameters based on whether or not we need to
  // increase them.
  // @param[in] increase - Whether or not to increase the regularization.
  void update_regularization_parameters(bool increase);

  // The number of steps we want to integrate and optimize controls over.
  std::size_t num_steps_{0U};

  DiscreteDynamics<State, Control> dynamics_;
  CostFunction<State, Control> cost_;

  DoubleBuffer<std::vector<State>> states_;
  DoubleBuffer<std::vector<Control>> controls_;

  std::vector<DynamicsDiffs<State, Control>> dynamics_diffs_;
  std::vector<CostDiffs<State, Control>> cost_diffs_;

  std::vector<VecU> feedforward_term_;
  std::vector<MatUX> feedback_term_;
  double optimal_cost_{0.};

  RegularizationParameters regularization_parameters_;
};

template <StateType State, ControlType Control>
ILQR<State, Control>::ILQR(
    const std::size_t num_steps,
    DiscreteDynamics<State, Control> dynamics,
    CostFunction<State, Control> cost)
    : num_steps_{num_steps},
      dynamics_{std::move(dynamics)},
      cost_{std::move(cost)},
      states_{
          std::vector<State>{num_steps_ + 1U},
          std::vector<State>{num_steps_ + 1U}},
      controls_{
          std::vector<Control>{num_steps_},
          std::vector<Control>{num_steps_}},
      dynamics_diffs_{num_steps_},
      cost_diffs_{num_steps_ + 1U},
      feedforward_term_(num_steps_, VecU::Zero()),
      feedback_term_(num_steps_, MatUX::Zero()) {
  REASSERT(num_steps > 0, "Must have at least one step!");
}

template <StateType State, ControlType Control>
double ILQR<State, Control>::forward_pass(const double linesearch_factor) {
  double cost = 0.0;
  for (std::size_t ii = 0; ii < num_steps_; ++ii) {
    controls_.mutable_next().at(ii) =
        controls_.current().at(ii) +
        (linesearch_factor * feedforward_term_.at(ii) +
         feedback_term_.at(ii) *
             (states_.next().at(ii) - states_.current().at(ii)));
    {
      const auto no_diffs = null_reference<DynamicsDiffs<State, Control>>;
      states_.mutable_next().at(ii + 1U) =
          dynamics_(states_.next().at(ii), controls_.next().at(ii), no_diffs);
    }
    {
      const auto no_diffs = null_reference<CostDiffs<State, Control>>;
      cost += cost_(
          states_.next().at(ii),
          NullableReference{controls_.next().at(ii)},
          no_diffs);
    }
  }

  // Don't forget to add the cost for the final state
  const auto no_diffs = null_reference<CostDiffs<State, Control>>;
  const auto no_control = null_reference<const Control>;
  cost += cost_(states_.next().at(num_steps_), no_control, no_diffs);
  return cost;
}

template <StateType State, ControlType Control>
void ILQR<State, Control>::compute_derivatives() {
  for (std::size_t ii = 0; ii < num_steps_; ++ii) {
    dynamics_(
        states_.current().at(ii),
        controls_.current().at(ii),
        NullableReference{dynamics_diffs_.at(ii)});
    cost_(
        states_.current().at(ii),
        NullableReference{controls_.current().at(ii)},
        NullableReference{cost_diffs_.at(ii)});
  }
  const auto no_control = null_reference<const Control>;
  cost_(
      states_.current().at(num_steps_),
      no_control,
      NullableReference{cost_diffs_.at(num_steps_)});
}

template <StateType State, ControlType Control>
void ILQR<State, Control>::backward_pass() {
  while (not try_backward_pass()) {
    constexpr bool INCREASE = true;
    update_regularization_parameters(INCREASE);
  }
  constexpr bool DECREASE = false;
  update_regularization_parameters(DECREASE);
}

template <StateType State, ControlType Control>
bool ILQR<State, Control>::try_backward_pass() {
  VecX V_x{cost_diffs_.at(num_steps_).cost_x};
  MatXX V_xx{cost_diffs_.at(num_steps_).cost_xx};
  for (std::size_t ii = num_steps_ - 1U; ii + 1U > 0U; --ii) {
    const CostDiffs<State, Control> &cost_diffs{cost_diffs_.at(ii)};
    const VecX &cost_x{cost_diffs.cost_x};
    const VecU &cost_u{cost_diffs.cost_u};
    const MatXX &cost_xx{cost_diffs.cost_xx};
    const MatUX &cost_ux{cost_diffs.cost_ux};
    const MatUU &cost_uu{cost_diffs.cost_uu};

    const DynamicsDiffs<State, Control> &dynamics_diffs{dynamics_diffs_.at(ii)};
    const MatXX &f_x{dynamics_diffs.f_x};
    const MatXU &f_u{dynamics_diffs.f_u};

    // Equations 5a-e
    const VecX Q_x = cost_x + f_x.transpose() * V_x;
    const VecU Q_u = cost_u + f_u.transpose() * V_x;
    MatXX Q_xx = cost_xx + f_x.transpose() * V_xx * f_x;
    const MatUX Q_ux = cost_ux + f_u.transpose() * V_xx * f_x;
    MatUU Q_uu = cost_uu + f_u.transpose() * V_xx * f_u;

    // Enforce symmetry
    constexpr double HALF = 0.5;
    Q_xx = HALF * Q_xx + HALF * Q_xx.transpose().eval();
    Q_uu = HALF * Q_uu + HALF * Q_uu.transpose().eval();

    // Regularization-specific quantities
    // Equations 10a-b
    const MatXX V_xx_reg{
        V_xx + regularization_parameters_.mu * MatXX::Identity()};
    const MatUX Q_ux_tilde = cost_ux + f_u.transpose() * V_xx_reg * f_x;
    MatUU Q_uu_tilde = cost_uu + f_u.transpose() * V_xx_reg * f_u;

    // Enforce symmetry
    Q_uu_tilde = HALF * Q_uu_tilde + HALF * Q_uu_tilde.transpose().eval();
    const auto llt = Q_uu_tilde.template selfadjointView<Eigen::Upper>().llt();
    if (llt.info() == Eigen::NumericalIssue) {
      // If we get here, then Q_uu_tilde is not PD and Eigen has accordingly
      // failed to find LL^T. We need a larger regularization term
      return false;
    }
    // Equations 10c-d
    feedforward_term_.at(ii) = -llt.solve(Q_u);
    feedback_term_.at(ii) = -llt.solve(Q_ux_tilde);

    const MatUX &K = feedback_term_.at(ii);
    const VecU &k = feedforward_term_.at(ii);

    // Equations 11b-c
    V_x = Q_x + K.transpose() * Q_uu * k + K.transpose() * Q_u +
          Q_ux.transpose() * k;
    V_xx = Q_xx + K.transpose() * Q_uu * K + K.transpose() * Q_ux +
           Q_ux.transpose() * K;

    // Enforce symmetry
    V_xx = HALF * V_xx + HALF * V_xx.transpose().eval();
  }
  return true;
}

template <StateType State, ControlType Control>
void ILQR<State, Control>::update_regularization_parameters(
    const bool increase) {
  auto &p = regularization_parameters_;

  // Expressions from Section F
  if (increase) {
    p.delta = std::max(1., p.delta) * p.delta_0;
    p.mu = std::max(p.mu_min, p.mu * p.delta);
  } else {
    p.delta = std::min(1., p.delta) / p.delta_0;
    p.mu *= p.delta;
    if (p.mu < p.mu_min) {
      p.mu = 0.;
    }
  }
}

template <StateType State, ControlType Control>
void ILQR<State, Control>::reset(
    const State &initial_state,
    const std::vector<Control> &initial_controls) {
  states_.mutable_next().at(0) = initial_state;
  states_.swap();
  states_.mutable_next().at(0) = initial_state;
  controls_.mutable_next() = initial_controls;
  controls_.swap();

  for (VecU &term : feedforward_term_) {
    term = VecU::Zero();
  }
  for (MatUX &term : feedback_term_) {
    term = MatUX::Zero();
  }

  // Reset the regularization parameters:
  regularization_parameters_ = {};
}

template <StateType State, ControlType Control>
typename ILQR<State, Control>::Result ILQR<State, Control>::optimize_controls(
    const State &initial_state,
    const std::vector<Control> &initial_controls,
    const std::size_t max_iterations,
    const std::size_t max_backtrack_iterations,
    const double tolerance) {
  constexpr auto ERR_MSG =
      "Initial controls size inconsistent with this object!";
  REASSERT(initial_controls.size() == num_steps_, ERR_MSG);

  reset(initial_state, initial_controls);
  constexpr double NO_LINESEARCH = 0.0;
  optimal_cost_ = forward_pass(NO_LINESEARCH);
  controls_.swap();
  states_.swap();

  bool converged = false;
  std::size_t iteration = 0U;
  for (; iteration < max_iterations and not converged; ++iteration) {
    compute_derivatives();
    backward_pass();

    double grad_squared = 0.0;
    for (const auto &k : feedforward_term_) {
      grad_squared += k.norm();
    }

    // https://en.wikipedia.org/wiki/Backtracking_line_search#Algorithm
    constexpr double DEFAULT_TAU = 0.7;
    struct LineSearchParameters {
      double c = 0.;
      double tau = DEFAULT_TAU;
    };
    constexpr LineSearchParameters LSP;
    double alpha = 1.0;
    double cost = 0.0;
    for (std::size_t backtrack_count = 0;
         backtrack_count < max_backtrack_iterations;
         ++backtrack_count) {
      cost = forward_pass(alpha);
      if (optimal_cost_ - cost >= -alpha * LSP.c * grad_squared) {
        break;
      }
      alpha *= LSP.tau;
    }
    controls_.swap();
    states_.swap();

    if (std::fabs(optimal_cost_ - cost) < tolerance) {
      converged = true;
    }
    optimal_cost_ = cost;
  }
  return {
      .converged = converged,
      .num_iterations = iteration + 1U,
      .cost = optimal_cost_,
      .states = states_.current(),
      .controls = controls_.current(),
  };
}

}  // namespace resim::planning
