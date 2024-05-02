// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <functional>

#include "resim/utils/nullable_reference.hh"

namespace resim::planning {

// The first differential of the discrete dynamics with respect to the state
// ("x") and the control ("u").
template <typename State, typename Control>
struct DynamicsDiffs {
  using MatXX = Eigen::Matrix<double, State::DIM, State::DIM>;
  using MatXU = Eigen::Matrix<double, State::DIM, Control::DIM>;
  MatXX f_x{MatXX::Zero()};
  MatXU f_u{MatXU::Zero()};

  void set_zero() {
    f_x.setZero();
    f_u.setZero();
  }
};

// The signature for a discrete dynamics function. The diffs are passed by a
// nullable reference since we don't always need to compute diffs, so
// NULL_REFERENCE should be passed when we don't care. Accordingly, the caller
// retains ownership of the differentials.
template <typename State, typename Control>
using DiscreteDynamics = std::function<State(
    const State &,
    const Control &,
    NullableReference<DynamicsDiffs<State, Control>>)>;

}  // namespace resim::planning
