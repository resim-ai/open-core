#pragma once

#include <concepts>
#include <type_traits>
#include <utility>
#include <variant>

namespace resim {
namespace detail {

// A typename template to collect all of our overloads into a single object
template <typename... Ts>
struct Overloads : public Ts... {
  using Ts::operator()...;
};

// Deduction guide
template <typename... Ts>
Overloads(Ts...) -> Overloads<Ts...>;

// A variable template determining whether a given type T is a variant.
template <typename T>
constexpr bool IS_VARIANT = false;

template <typename... Ts>
constexpr bool IS_VARIANT<std::variant<Ts...>> = true;

// A concept reflecting whether a given type represents a variant, or a
// reference to one, possibly with cv-qualification.
template <typename T>
concept VariantType = IS_VARIANT<std::decay_t<T>>;

// A concept reflecting whether a given type represents an typename, or a
// reference to one, possibly with cv-qualification.
template <typename T>
concept ClassType = std::is_class_v<std::decay_t<T>>;

// The following definitions are designed to check the consistency of the
// branches passed in and result in a concept that allows us to discard this
// function from the overload set when these aren't consistent.

// A helper variable template determining whether the given branches are
// collectively invocable on all variant cases. In other words, all cases have
// exactly one best function that can be called on them.
template <typename Variant, typename... BranchTypes>
constexpr bool ALL_CASES_INVOCABLE = false;

template <typename... Ts, typename... BranchTypes>
constexpr bool ALL_CASES_INVOCABLE<std::variant<Ts...>, BranchTypes...> =
    (std::is_invocable_v<Overloads<BranchTypes...>, Ts> and ...);

// A simple helper to determine whether a parameter pack contains all the same
// types.
template <typename... Ts>
constexpr bool ALL_SAME = true;  // Base case for empty packs

template <typename Head, typename... Tail>
constexpr bool ALL_SAME<Head, Tail...> = (std::is_same_v<Head, Tail> and ...);

// A helper variable template to determine whether all invocations of the
// branches on each case of the variant yield the same return type.
template <typename T, typename... BranchTypes>
constexpr bool ALL_INVOCATIONS_MATCH = false;

template <typename... Ts, typename... BranchTypes>
constexpr bool ALL_INVOCATIONS_MATCH<std::variant<Ts...>, BranchTypes...> =
    ALL_SAME<std::invoke_result_t<Overloads<BranchTypes...>, Ts>...>;

// The final concept determining whether the branches are consistent with the
// variant and each other. We have to decay since T and BranchTypes may be
// deduced by match() below with references or cv qualifiers.
template <typename T, typename... BranchTypes>
concept BranchesConsistent =
    ALL_CASES_INVOCABLE<std::decay_t<T>, std::decay_t<BranchTypes>...> &&
    ALL_INVOCATIONS_MATCH<std::decay_t<T>, std::decay_t<BranchTypes>...>;

}  // namespace detail

// This function implements a pattern matching functionality for variants. Users
// provide the variant along with a series of functors. A functor is chosen from
// this list using the overload resolution rules based on the current type held
// by the variant. All possible types held by the variant must be handled by at
// least one branch functor. See match.md for more information and an example.
// @param[in] variant - The variant to match.
// @param[in] branch - The branches to match against. All branches must return
//                     the same return type. Do not use functors with non-const
//                     operator() because this can yield non-intuitive results.
//  @returns Whatever the selected branch returns.
template <detail::VariantType T, detail::ClassType... BranchTypes>
requires detail::BranchesConsistent<T, BranchTypes...>
auto match(T &&variant, BranchTypes &&...branch) -> decltype(auto) {
  return std::visit(
      detail::Overloads{std::forward<BranchTypes>(branch)...},
      std::forward<T>(variant));
}

}  // namespace resim
