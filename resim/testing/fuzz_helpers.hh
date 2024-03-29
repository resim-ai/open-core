// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/timestamp.pb.h>

#include <algorithm>
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
// ADL when called in generic code.
template <std::floating_point Float, typename Rng>
Float random_element(TypeTag<Float> /*unused*/, InOut<Rng> rng) {
  constexpr double TWO = 2.;
  std::uniform_real_distribution<Float> dist{
      -std::numeric_limits<Float>::max() / TWO,
      std::numeric_limits<Float>::max() / TWO};
  return dist(*rng);
}

// verify_equality overload with an implementation for doubles.
bool verify_equality(double a, double b);

// Random element for protobuf timestamps
template <typename Rng>
google::protobuf::Timestamp random_element(
    TypeTag<google::protobuf::Timestamp> /*unused*/,
    InOut<Rng> rng) {
  google::protobuf::Timestamp result;
  result.set_seconds(random_element(TypeTag<int32_t>(), rng));
  constexpr int32_t NANOS_LB = 0;
  constexpr int32_t NANOS_UB = 1000000000;
  std::uniform_int_distribution<int32_t> dist{NANOS_LB, NANOS_UB};
  result.set_nanos(dist(*rng));
  return result;
}

bool verify_equality(
    const google::protobuf::Timestamp &a,
    const google::protobuf::Timestamp &b);

// Generate a random vector of T between size 3 and 7 assuming that
// random_element(TypeTag<T>, InOut<Rng>) has been defined either in the resim
// namespace or the same namespace as T. The size bounds were selected as
// between 3 and 7 so we could have a range of sizes represented without slowing
// tests down with very large vectors of elements to generate and check.
template <typename T, typename Rng>
std::vector<T> random_element(
    TypeTag<std::vector<T>> /*unused*/,
    InOut<Rng> rng) {
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  std::vector<T> result;
  result.reserve(size);
  for (size_t ii = 0; ii < size; ++ii) {
    result.emplace_back(random_element(TypeTag<T>(), rng));
  }
  return result;
}

// Verify that two vectors of type T are equal assuming that
// verify_equality(const T &a, const T &b) is defined either in the resim
// namespace or the same namespace as T.
template <typename T>
bool verify_equality(const std::vector<T> &a, const std::vector<T> &b) {
  // Requires C++14 to not be undefined behavior. Thanksfully, the CTAD on the
  // righthand-side of the equality requires C++17.
  return std::mismatch(
             a.cbegin(),
             a.cend(),
             b.cbegin(),
             b.cend(),
             [](const T &a, const T &b) { return verify_equality(a, b); }) ==
         std::pair(a.cend(), b.cend());
}

}  // namespace resim
