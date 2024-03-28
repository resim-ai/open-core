// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <utility>

#include "resim/dynamics/controller.hh"
#include "resim/dynamics/dynamics.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::dynamics {

//
// This interface describes a generic integrator that works with a given
// Dynamics class and a given Controller class. In particular, given a time
// increment (dt), a system state, the time, its dynamics, and its controller,
// an integrator tells you what the state should be at time + dt. Alternatively,
// if a user wants they may use implementations of this interface directly with
// a CompleteDynamics function to directly integrate ODEs of the form:
//
//                                 x' = f(x, t).
//
// Since most integrators are defined for systems like the above, the direct
// CompleteDynamics overload is the only virtual one that implementations need
// to override. The Dynamics + Controller overload just uses the direct overload
// under the hood.
//
template <StateType State_t>
class Integrator {
 public:
  static constexpr int DOF = State_t::DOF;
  using State = State_t;
  using Delta = Eigen::Matrix<double, DOF, 1>;
  using MatXX = Eigen::Matrix<double, DOF, DOF>;

  Integrator() = default;
  Integrator(const Integrator &) = default;
  Integrator(Integrator &&) noexcept = default;
  Integrator &operator=(const Integrator &) = default;
  Integrator &operator=(Integrator &&) noexcept = default;
  virtual ~Integrator() = default;

  // This function takes a time increment, the current state, the current time,
  // along with the system dynamics and controller, and returns what the state
  // should be at time + dt according to this integration scheme. This is a
  // convenience overload which takes a dynamics and controller and couples them
  // into a CompleteDynamics used with the overload below. This is not constant
  // because some integrators are stateful.
  // @param[in] dt - The timestep to take.
  // @param[in] state - The current state.
  // @param[in] time - The current time.
  // @param[in] dynamics - The system dynamics.
  // @param[in] controller - The system controller.
  // @returns The state predicted by the integrator at time + dt.
  template <int CONTROL_DIM>
  State operator()(
      time::Duration dt,
      const State &state,
      time::Timestamp time,
      const Dynamics<State, CONTROL_DIM> &dynamics,
      const Controller<State, CONTROL_DIM> &controller);

  using CompleteDynamics = std::function<
      Delta(const State &, time::Timestamp, NullableReference<MatXX>)>;

  // This function takes a time increment, the current state, the current time,
  // along with the complete system dynamics, and returns what the state should
  // be at time + dt according to this integration scheme. This is not constant
  // because some integrators are stateful.
  // @param[in] dt - The timestep to take.
  // @param[in] state - The current state.
  // @param[in] time - The current time.
  // @param[in] complete_dynamics - The complete system dynamics.
  // @returns The state predicted by the integrator at time + dt.
  virtual State operator()(
      time::Duration dt,
      const State &state,
      time::Timestamp time,
      const CompleteDynamics &complete_dynamics) = 0;
};

template <StateType State>
template <int CONTROL_DIM>
State Integrator<State>::operator()(
    const time::Duration dt,
    const State &state,
    const time::Timestamp time,
    const Dynamics<State, CONTROL_DIM> &dynamics,
    const Controller<State, CONTROL_DIM> &controller) {
  using DynamicsType = Dynamics<State, CONTROL_DIM>;
  using ControllerType = Controller<State, CONTROL_DIM>;
  using DynamicsDiffs = typename DynamicsType::Diffs;
  using ControllerJacobian = typename ControllerType::Jacobian;
  return (*this)(
      dt,
      state,
      time,
      [&dynamics, &controller](
          const State &state,
          const time::Timestamp time,
          NullableReference<MatXX> jacobian) {
        if (jacobian.has_value()) {
          DynamicsDiffs diffs;
          ControllerJacobian controller_jacobian;

          Delta delta{dynamics(
              state,
              controller(state, time, NullableReference{controller_jacobian}),
              time,
              NullableReference{diffs})};

          // The chain rule:
          *jacobian = diffs.f_x + diffs.f_u * controller_jacobian;
          return std::move(delta);
        }

        return dynamics(
            state,
            controller(state, time, null_reference),
            time,
            null_reference);
      });
}

}  // namespace resim::dynamics
