#pragma once

#include <Eigen/Dense>
#include <functional>

#include "resim_core/utils/nullable_reference.hh"

namespace resim::planning {

// The first and second differentials of a cost function with respect to the
// state ("x") and the control ("u").
template <typename State, typename Control>
struct CostDiffs {
  using VecX = Eigen::Matrix<double, State::DIM, 1>;
  using VecU = Eigen::Matrix<double, Control::DIM, 1>;
  using MatXX = Eigen::Matrix<double, State::DIM, State::DIM>;
  using MatUX = Eigen::Matrix<double, Control::DIM, State::DIM>;
  using MatUU = Eigen::Matrix<double, Control::DIM, Control::DIM>;

  VecX cost_x{VecX::Zero()};
  VecU cost_u{VecU::Zero()};
  MatXX cost_xx{MatXX::Zero()};
  MatUX cost_ux{MatUX::Zero()};
  MatUU cost_uu{MatUU::Zero()};
};

// The signature for a cost function. The control is passed as a nullable
// reference since the cost does not always depend on control (e.g. the last
// step in iLQR). The diffs are passed as a nullable reference since we don't
// always need to compute diffs, so NULL_REFERENCE should be passed when we
// don't care. Accordingly, the caller retains ownership of both of these
// arguments.
template <typename State, typename Control>
using CostFunction = std::function<double(
    const State &,
    NullableReference<const Control>,
    NullableReference<CostDiffs<State, Control>>)>;

}  // namespace resim::planning
