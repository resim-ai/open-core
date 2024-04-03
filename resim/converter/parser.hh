
#pragma once

#include <tuple>
#include <type_traits>

#include "resim/converter/tags.hh"

namespace resim::converter {

template <typename T, typename Getters>
class Parser {
 public:
  constexpr explicit Parser(const Getters &getters) : getters_{getters} {}

  static constexpr std::size_t NUM_FIELDS = std::tuple_size_v<Getters>;

  template <std::size_t Idx>
  using FieldType =
      std::remove_reference_t<std::remove_cv_t<decltype(std::get<Idx>(
          std::declval<Getters>())(std::declval<T &>()))>>;

  template <std::size_t Idx>
  auto get(auto &s) const -> decltype(auto) {
    return std::get<Idx>(getters_)(s);
  }

 private:
  Getters getters_;
};

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

#define POD_GETTER(field) [](auto &s) -> decltype(auto) { return s.field; }

#define PROTO_GETTER(field)                                                \
  [](auto &s) -> decltype(auto) {                                          \
    if constexpr (std::is_const_v<std::remove_reference_t<decltype(s)>>) { \
      return s.field();                                                    \
    } else {                                                               \
      return *s.mutable_##field();                                         \
    }                                                                      \
  }

#define DEFINE_GET_PARSER(struct_name, ...)                                \
  constexpr auto get_parser(::resim::converter::TypeTag<struct_name> tt) { \
    return ::resim::converter::make_parser<struct_name>(                   \
        std::make_tuple(__VA_ARGS__));                                     \
  }

}  // namespace resim::converter
