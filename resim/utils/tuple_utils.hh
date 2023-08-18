// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <tuple>
#include <utility>

#include "resim/utils/type.hh"

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
// propagate to values. Further note the & & -> & and & && -> & collapsing rules
// at play. For other examples, refer to tuple_utils_test.cc. This shouldn't
// typically be a concern as tuples containing references are not incredibly
// common.
template <typename Callable, typename Tuple>
auto for_each_in_tuple(const Callable &f, Tuple &&t) {
  return std::apply(
      [&f](auto &&...elements) {
        // clang-format off
        using ResultType = 
            std::tuple<
              decltype(f(
                std::forward<decltype(elements)>(elements)
              ))...>;

        return ResultType(
            f(
              std::forward<decltype(elements)>(elements)
            )...);
        // clang-format on
      },
      std::forward<Tuple>(t));
}

// This constexpr helper function takes two index sequences and returns the
// concatenation of the two.
template <std::size_t... Idxs1, std::size_t... Idxs2>
constexpr auto index_sequence_cat(
    std::index_sequence<Idxs1...> /*unused*/,
    std::index_sequence<Idxs2...> /*unused*/) {
  return std::index_sequence<Idxs1..., Idxs2...>();
}

// This struct contains the implementation for the filter_tuple function below.
// The goal of this function is to remove all entries from a given tuple where
// the entry's type does not satisfy a given boolean type trait. For example,
// running filter_tuple<std::is_integral>(std::make_tuple(1, "chair",
// std::unique_ptr<int>())) should output a std::tuple<int> containing (1,).
//
// The implementation is here to perform the following steps:
//
// At compile time, the constexpr member function get_indices() uses tail-end
// recursion to get the indices of the given tuple which satisfy the given
// FilterTrait (i.e. FilterTrait<std::tuple_element_t<Index>>::value == true).
// These are collected in an index sequence which is used with get_tuple() at
// runtime in operator() to extract out the elements from the tuple with those
// indices.
template <template <typename> typename FilterTrait>
struct FilterTupleImpl {
  template <std::size_t IDX>
  auto get_indices() {
    return std::index_sequence<>();
  }

  template <
      std::size_t IDX,
      typename Head,
      typename... Tail,
      std::enable_if_t<FilterTrait<Head>::value, bool> = true>
  auto get_indices(BasicType<Head> /*unused*/, BasicType<Tail>... /*unused*/) {
    return index_sequence_cat(
        std::index_sequence<IDX>(),
        get_indices<IDX + 1U>(TypeC<Tail>...));
  }

  template <
      std::size_t IDX,
      typename Head,
      typename... Tail,
      std::enable_if_t<not FilterTrait<Head>::value, bool> = true>
  auto get_indices(BasicType<Head> /*unused*/, BasicType<Tail>... /*unused*/) {
    return get_indices<IDX + 1U>(TypeC<Tail>...);
  }

  // Convenient entrypoint for this function which immediately jumps into the
  // recursion.
  template <std::size_t IDX, typename... Ts>
  auto get_indices(const std::tuple<Ts...> & /*unused*/) {
    return get_indices<IDX>(TypeC<Ts>...);
  }

  template <typename Tuple, std::size_t... Idxs>
  auto get_tuple(Tuple &&t, std::index_sequence<Idxs...> /*unused*/) {
    return std::tuple<std::tuple_element_t<
        Idxs,
        std::remove_cv_t<std::remove_reference_t<Tuple>>>...>(
        std::get<Idxs>(std::forward<Tuple>(t))...);
  }

  template <typename Tuple>
  auto operator()(Tuple &&t) {
    constexpr auto INDICES = decltype(get_indices<0U>(t))();
    return get_tuple(std::forward<Tuple>(t), INDICES);
  }
};

// Filter out the elements of a given tuple which satisfy FilterTrait. In other
// words, remove elements for which FilterTrait<T>::value == false, leaving
// behind a smaller tuple. The resulting tuple copies the elements of the input
// tuple if it is an lvalue, and moves the elements if it is an rvalue. It must
// therefore have copyable elements if it is an lvalue. For instance,
// a std::tuple<int &&> lvalue cannot be filtered.
template <template <typename> typename FilterTrait, typename Tuple>
auto filter_tuple(Tuple &&tuple) {
  return FilterTupleImpl<FilterTrait>()(std::forward<Tuple>(tuple));
}

}  // namespace resim
