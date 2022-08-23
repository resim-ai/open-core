#pragma once

#include <random>
#include <utility>

namespace resim::testing {

// This function template generates psuedo-random vectors with independent
// entries for testing purposes.
// @tparam[in] Vector_t - The vector type to generate. Assumed to be an Eigen
//                        type.
// @param[in] generator - The pseudo random number generator to use
//                        (e.g. std::mt19937).
// @param[in] distribution - The distribution to use
//                           (e.g. std::uniform_real_distribution).
template <
    typename Vector_t,
    typename RandomGenerator_t,
    typename Distribution_t>
Vector_t random_vector(
    RandomGenerator_t &&generator,
    Distribution_t &&distribution) {
  return Vector_t::NullaryExpr([&]() {
    return std::forward<Distribution_t>(distribution)(
        std::forward<RandomGenerator_t>(generator));
  });
}

// Overload of the above which defaults to uniform real distribution between
// -1.0 and 1.0.
// @tparam[in] Vector_t - The vector type to generate. Assumed to be an Eigen
//                        type.
// @param[in] generator - The pseudo random number generator to use
//                        (e.g. std::mt19937).
template <typename Vector_t, typename RandomGenerator_t>
Vector_t random_vector(RandomGenerator_t &&generator) {
  using Scalar = typename Vector_t::Scalar;
  using Distribution = std::uniform_real_distribution<Scalar>;
  constexpr Scalar LB = -1.0;
  constexpr Scalar UB = 1.0;
  Distribution dist{LB, UB};
  return random_vector<Vector_t>(generator, dist);
}

}  // namespace resim::testing
