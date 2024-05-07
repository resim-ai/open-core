// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>
#include <unordered_map>

#include "resim/planning/cost_function.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::planning {

template <typename State, typename Control>
class CostFunctionRegistry {
 public:
  using Cost = CostFunction<State, Control>;

  Cost &operator[](std::string key);

  double operator()(
      const State &x,
      NullableReference<const Control> u,
      NullableReference<CostDiffs<State, Control>> diffs) const;

 private:
  std::unordered_map<std::string, Cost> functions_;
};

template <typename State, typename Control>
CostFunction<State, Control> &CostFunctionRegistry<State, Control>::operator[](
    std::string key) {
  return functions_[key];
}

template <typename State, typename Control>
double CostFunctionRegistry<State, Control>::operator()(
    const State &x,
    NullableReference<const Control> u,
    NullableReference<CostDiffs<State, Control>> diffs) const {
  double cost_sum = 0.0;
  for (const auto &[_, cost_fn] : functions_) {
    cost_sum += cost_fn(x, u, diffs);
  }
  return cost_sum;
}

}  // namespace resim::planning
