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
// disables ADL. See the documentation for random_element() below for
// more information on why this is important.
template <typename T>
struct TypeTag {};

// Function to get a randomly-generated instance of a given type. We don't
// actually define this, but we provide a declaration so that generic algorithms
// can see that such a function should exist. This is in order to support
// a pattern which allows for composition of random_element() generators to form
// generators for more and more complicated types.
//
// Here's how it works:
//
//  - When users write a new struct or class `T`, they can define a function
//    `T random_element(TypeTag<T>, InOut<Rng> rng)` for their type which can
//    be used to generate random instances of that class. For instance, for
//    a user-defined `Animal` type:
//
//    template <typename Rng>
//    Animal random_element(
//        TypeTag<Animal> /*unused*/,
//        InOut<Rng> rng) {
//        return Animal{ /* Initialize with rng */ };
//    }
//
//  - Then, generic algorithms (even those defined above `T
//    random_element(TypeTag<T>, InOut<Rng> rng)` can use this function using
//    argument dependent lookup (ADL). Specifically, they can call
//    `random_element(TypeTag<T>(), rng)` (with no namespace qualification).
//    During name lookup, the compiler will then search the namespace of
//    `TypeTag` and the namespace of `T` for any functions called
//    `random_element()` to participate in overload resolution. This will find
//    the user's function provided it's defined in the same namespace as `T`.
//
//  - Some types (e.g. int, or std::string) are in the global namespace or
//    namespaces we don't want to modify, so we introduce `random_element()`
//    definitions for them in the same namespace as `TypeTag` (since that
//    namespace is also searched). One such example is the overload of
//    random_element() for integer types below.
//
//  - This allows users to take advantage of common generic implementations of
//    things like `std::vector<T> random_element(TypeTag<std::vector<T>>,
//    InOut<Rng>)` for any `T` with `T random_element(TypeTag<T>, InOut<Rng>)`
//    defined.
//    TODO(mikebauer) reference this example when it exists.
//
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

// random_element overload with an implementation for floating point types. This
// is done here so it is in the same namespace as TypeTag and can be found by
// ADL
// when called in generic code.
template <std::floating_point Float, typename Rng>
Float random_element(TypeTag<Float> /*unused*/, InOut<Rng> rng) {
  std::uniform_real_distribution<Float> dist{
      std::numeric_limits<Float>::min(),
      std::numeric_limits<Float>::max()};
  return dist(*rng);
}

// verify_equality overload with an implementation for doubles.
bool verify_equality(double a, double b);

}  // namespace resim
