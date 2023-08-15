// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <tuple>

namespace resim {

// This function executes a given Callable object on each element of the given
// tuple, storing the results of each invocation in a tuple of the same length
// as the input tuple. The Callable must return something for each input type
// from the tuple. Take great care when using this function on tuples containing
// references as the rules are complex. The short version is that the cv
// qualifiers and references associated with the tuple type are collapsed with
// those of each element when passed into the callable. For example:
//
// A tuple of type const tuple<T, T &, const T &, T &&, const T &&> will result
// in the following calls to f:
//
// f(const T &),
// f(T &),
// f(const T &),
// f(T &),
// f(const T &),
//
// Note that the const from the tuple has no effect on references while it does
// propgate to values. Further note the & & -> & and & && -> & collapsing rules
// at play. For other examples, refer to tuple_utils_test.cc. This shouldn't
// typically be a concern as tuples containing references are not incredibly
// common.
template <typename Callable, typename Tuple>
auto for_each_in_tuple(const Callable &f, Tuple &&t) {
  return std::apply(
      [&f](auto &&...elements) {
        using ResultType = std::tuple<decltype(f(
            std::forward<decltype(elements)>(elements)))...>;
        return ResultType(f(std::forward<decltype(elements)>(elements))...);
      },
      std::forward<Tuple>(t));
}

}  // namespace resim
