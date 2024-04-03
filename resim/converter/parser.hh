// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <tuple>
#include <type_traits>
#include <utility>

#include "resim/converter/tags.hh"
#include "resim/utils/setter_reference.hh"

namespace resim::converter {

// This class template describes the getters available for a type T and thereby
// allows for automatic manipulations of that type (e.g. fuzz helper
// generation). Typically, we define a function called get_parser() for a type
// (using the macros below) in order to get this object.
template <typename T, typename Getters>
class Parser {
 public:
  constexpr explicit Parser(Getters getters) : getters_{std::move(getters)} {}

  static constexpr std::size_t NUM_FIELDS = std::tuple_size_v<Getters>;

  template <std::size_t Idx>
  using FieldType =
      std::remove_cv_t<std::remove_reference_t<decltype(std::get<Idx>(
          std::declval<Getters>())(std::declval<const T &>()))>>;

  // The actualy getter
  template <std::size_t Idx>
  auto get(auto &s) const -> decltype(auto) {
    return std::get<Idx>(getters_)(s);
  }

 private:
  Getters getters_;
};

// Define a concept to tell whether a type T has get_parser() defined for it.
namespace detail {
template <typename T>
struct IsParser : std::false_type {};

template <typename T, typename... Getters>
struct IsParser<Parser<T, Getters...>> : std::true_type {};

template <typename T>
concept IsParserConcept = IsParser<T>::value;

}  // namespace detail

template <typename T>
concept Parsable = requires(T) {
  { get_parser(TypeTag<T>()) } -> detail::IsParserConcept;
};

template <typename T, typename Getters>
auto make_parser(const Getters &getters) {
  return Parser<T, Getters>(getters);
}

// Defines a getter for a field in a POD data structure
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define POD_GETTER(field) [](auto &s) -> decltype(auto) { return s.field; }

// Defines a getter for a non-primitive field in a generated protobuf C++
// object.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PROTO_GETTER(field)                                                \
  [](auto &s) -> decltype(auto) {                                          \
    if constexpr (std::is_const_v<std::remove_reference_t<decltype(s)>>) { \
      return s.field();                                                    \
    } else {                                                               \
      return *s.mutable_##field();                                         \
    }                                                                      \
  }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PROTO_PRIMITIVE_GETTER(field)                                      \
  [](auto &s) -> decltype(auto) {                                          \
    if constexpr (std::is_const_v<std::remove_reference_t<decltype(s)>>) { \
      return s.field();                                                    \
    } else {                                                               \
      return resim::SetterReference(                                       \
          [&s](const auto &val) { return s.set_##field(val); },            \
          [&s]() -> decltype(auto) { return s.field(); });                 \
    }                                                                      \
  }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DEFINE_GET_PARSER(struct_name, ...)                                \
  constexpr auto get_parser(::resim::converter::TypeTag<struct_name> tt) { \
    return ::resim::converter::make_parser<struct_name>(                   \
        std::make_tuple(__VA_ARGS__));                                     \
  }

}  // namespace resim::converter
