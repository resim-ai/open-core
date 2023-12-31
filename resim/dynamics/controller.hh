// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/dynamics/dynamics.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::dynamics {

//
// This interface describes the control for a system of ordinary differential
// equations (ODEs). In other words, classes that implement this interface
// represent u in the system of ODEs:
//
//                     (d/dt)[state] = f(state, control, time)
//                           control = u(state, time)
//
// The interface also gives implementers the ability to add Jacobian information
// that can be useful for integrating such systems implicitly. Not all
// controllers are expected to implement this Jacobian functionality. This is a
// class because such controllers are often parameterized by things like,
// e.g. proportional controller gains.
//
template <StateType State_t, int CONTROL_DIM>
class Controller {
 public:
  using State = State_t;
  using Control = Eigen::Matrix<double, CONTROL_DIM, 1>;
  using Jacobian = Eigen::Matrix<double, CONTROL_DIM, State::DOF>;

  Controller() = default;
  Controller(const Controller &) = default;
  Controller(Controller &&) noexcept = default;
  Controller &operator=(const Controller &) = default;
  Controller &operator=(Controller &&) noexcept = default;
  virtual ~Controller() = default;

  // The controller itself.
  // @param[in] state - The state of the system.
  // @param[in] time - The current time.
  // @param[out] jacobian - The Jacobian of the control vector w.r.t. state.
  //                        This should be populated if not null.
  virtual Control operator()(
      const State &state,
      time::Timestamp time,
      NullableReference<Jacobian> jacobian) const = 0;
};

}  // namespace resim::dynamics
