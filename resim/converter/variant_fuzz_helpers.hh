
#pragma once

#include <cstddef>
#include <optional>
#include <random>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

#include "resim/assert/assert.hh"
#include "resim/converter/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::converter {

namespace detail {
template <typename... Ts, std::size_t... Idxs, typename Rng>
void set_variant_element_randomly(
    const std::size_t index_to_set,
    std::index_sequence<Idxs...>,
    InOut<std::optional<std::variant<Ts...>>> variant,
    InOut<Rng> rng) {
  const auto maybe_set = [&](auto index_object) {
    constexpr std::size_t INDEX = decltype(index_object)::value;
    using ElementType = std::tuple_element_t<INDEX, std::tuple<Ts...>>;
    if (index_to_set == INDEX) {
      *variant = std::variant<Ts...>(
          std::in_place_index<INDEX>,
          ::resim::converter::random_element<ElementType>(rng));
    }
  };

  (void)((maybe_set(std::integral_constant<std::size_t, Idxs>()), true) && ...);
}

template <typename... Ts, std::size_t... Idxs>
bool variantify_equality(
    std::index_sequence<Idxs...>,
    const std::variant<Ts...> &a,
    const std::variant<Ts...> &b) {
  if (a.index() != b.index()) {
    return false;
  }

  const auto is_equal = [&](auto index_object) {
    constexpr std::size_t INDEX = decltype(index_object)::value;
    if (a.index() == INDEX) {
      return ::resim::converter::verify_equality(
          std::get<INDEX>(a),
          std::get<INDEX>(b));
    }
    // nada == nada
    return true;
  };
  return (is_equal(std::integral_constant<std::size_t, Idxs>()) && ...);
}

}  // namespace detail

template <typename... Ts, typename Rng>
std::variant<Ts...> random_element(
    TypeTag<std::variant<Ts...>> /*unused*/,
    InOut<Rng> rng) {
  std::uniform_int_distribution<std::size_t> dist{0, sizeof...(Ts) - 1};
  const std::size_t index = dist(*rng);

  // Use an optional since the variant might not be default constructable.
  std::optional<std::variant<Ts...>> result;
  detail::set_variant_element_randomly(
      index,
      std::index_sequence_for<Ts...>(),
      InOut{result},
      rng);
  REASSERT(result.has_value());
  return *result;
}

template <typename... Ts>
bool verify_equality(
    ADLTag /*unused*/,
    const std::variant<Ts...> &a,
    const std::variant<Ts...> &b) {
  return detail::variantify_equality(std::index_sequence_for<Ts...>(), a, b);
}

}  // namespace resim::converter
