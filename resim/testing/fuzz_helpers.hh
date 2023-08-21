// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <concepts>
#include <random>

#include "resim/utils/inout.hh"

namespace resim {

// A type tag that allows us to find helpers via argument-dependent lookup
// (ADL). Note that we aren't using resim::Type since that template explicitly
// disables ADL. The utility of using ADL here is that it becomes possible to
// write generic helpers (i.e. ones that work on vectors of any type T. For this
// to work, the helpers always need to be defined in the same namspace as T *or*
// in the same namespace as this tag.
template <typename T>
struct TypeTag {};

// Function to get a randomly-generated instance of a given type.
template <typename T, typename Rng>
T random_element(TypeTag<T>, InOut<Rng> rng);

// Overload which allows users to get a random element with
// random_element<T>(rng) while still taking advantage of ADL.
template <typename T, typename Rng>
T random_element(InOut<Rng> rng) {
  return random_element(TypeTag<T>(), rng);
}

// Function to verify the equality of two elements of type T in a unit test.
template <typename T>
bool verify_equality(const T &a, const T &b);

// random_element overload with an implementation for integer types. This is
// done here so it is in the same namespace as TypeTag and can be found by ADL
// when called in generic code.
template <std::integral Int, typename Rng>
Int random_element(TypeTag<Int> /*unused*/, InOut<Rng> rng) {
  std::uniform_int_distribution<Int> dist{
      std::numeric_limits<Int>::min(),
      std::numeric_limits<Int>::max()};
  return dist(*rng);
}

}  // namespace resim
