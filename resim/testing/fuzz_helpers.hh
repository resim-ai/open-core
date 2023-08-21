
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

template <typename T, typename Rng>
T random_element(TypeTag<T>, InOut<Rng> rng);

template <typename T, typename Rng>
T random_element(InOut<Rng> rng) {
  return random_element(TypeTag<T>(), rng);
}

template <typename T>
bool verify_equality(const T &a, const T &b);

template <std::integral Int, typename Rng>
Int random_element(TypeTag<Int> /*unused*/, InOut<Rng> rng) {
  std::uniform_int_distribution<Int> dist{
      std::numeric_limits<Int>::min(),
      std::numeric_limits<Int>::max()};
  return dist(*rng);
}

}  // namespace resim
