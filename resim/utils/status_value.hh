// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <type_traits>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/utils/match.hh"
#include "resim/utils/status.hh"

namespace resim {

template <class T>
class StatusValue {
 public:
  static_assert(
      not std::is_same_v<std::decay<T>, Status>,
      "StatusValue<Status> not allowed!");

  using ValueType = T;

  // We claim that this is one of the few places where the benefits of allowing
  // implicit conversions is worth the cost. In particular, we want to allow
  // implicit conversions from Status objects, and from the value type held in
  // this StatusValue. The argument for implicit conversions from Status and T
  // is that we want this type to have the same semantics as std::variant, so
  // we can do things like:
  //
  // StatusValue<int> my_function() {
  //   if (failed()) {
  //     return MAKE_STATUS("Oh no!");
  //   }
  //   return 3;
  // }
  //
  // The implicit conversion from Status also allows RETURN_IF_NOT_OK() and
  // RETURN_OR_ASSIGN() (defined below) to be employed in functions returning
  // StatusResult<T> for any type T. Otherwise, the desired return type would
  // have to somehow be passed into the macros as well.
  //
  // NOLINTBEGIN(google-explicit-constructor)

  // Perfect forwarding constructor for the value held in this StatusValue. We
  // require that ValueT to have the same decayed type as T to keep this from
  // competing with the other constructors for overload preference. We could use
  // constraints for this since we're in C++20, but clang-tidy doesn't like that
  // yet, so we used SFINAE with enable_if instead. See the following link for
  // more info on why we need to guard this:
  // https://releases.llvm.org/11.1.0/tools/clang/tools/extra/docs/clang-tidy/checks/bugprone-forwarding-reference-overload.html
  template <
      typename ValueT,
      typename = std::enable_if_t<
          std::is_same_v<std::decay_t<ValueT>, std::decay_t<T>>>>
  constexpr StatusValue(ValueT &&value);

  constexpr StatusValue(const Status &status);
  // NOLINTEND(google-explicit-constructor)

  constexpr StatusValue();
  ~StatusValue() = default;
  StatusValue(const StatusValue<T> &) = default;
  StatusValue(StatusValue<T> &&) noexcept = default;
  StatusValue &operator=(const StatusValue<T> &) = default;
  StatusValue &operator=(StatusValue<T> &&) noexcept = default;

  // Whether this StatusValue is okay, meaning it contains a value rather than a
  // bad status.
  constexpr bool ok() const;

  // Getters
  // Note that we have decided not to allow mutable access to the underlying
  // status or value for lvalue objects of type StatusValue<T>. This means that
  // users can't modify the contents of a StatusResult once it is constructed.

  // Return the value if present, or throw an exception if not.
  constexpr const T &value() const &;
  constexpr T &&value() &&;

  // Return the status if it's bad, or OKAY_STATUS if it's not.
  constexpr const Status &status() const;

 private:
  // Use a wrapper so T can be a reference
  struct ValueWrapper {
    // Perfect forwarding constructor to prevent any extra moves/copies when
    // users are populating.
    template <typename... Args>
    explicit constexpr ValueWrapper(Args &&...args)
        : value{std::forward<Args>(args)...} {}
    T value;
  };

  static constexpr auto VALUELESS_ERR =
      "Can't make okay StatusValue without value!";

  std::variant<ValueWrapper, Status> value_or_status_;
};

template <typename T>
constexpr StatusValue<T>::StatusValue()
    : value_or_status_{MAKE_STATUS("Uninitialized StatusValue!")} {}

template <typename T>
template <typename ValueT, typename>
constexpr StatusValue<T>::StatusValue(ValueT &&value)
    : value_or_status_{
          std::in_place_type<ValueWrapper>,  // Needed to avoid an extra move
                                             // under certain circumstances
          std::forward<ValueT>(value)} {}

template <typename T>
constexpr StatusValue<T>::StatusValue(const Status &status)
    : value_or_status_{status} {
  REASSERT(not ok(), VALUELESS_ERR);
}

template <typename T>
constexpr bool StatusValue<T>::ok() const {
  return status().ok();
}

template <typename T>
constexpr const T &StatusValue<T>::value() const & {
  REASSERT(ok(), "Can't dereference bad StatusValue!");
  return std::get<ValueWrapper>(value_or_status_).value;
}

template <typename T>
constexpr T &&StatusValue<T>::value() && {
  REASSERT(ok(), "Can't dereference bad StatusValue!");
  // We use forward here so we move if and only if T is a non-const rvalue
  // reference
  return std::forward<T>(std::get<ValueWrapper>(value_or_status_).value);
}

template <typename T>
constexpr const Status &StatusValue<T>::status() const {
  return match(
      value_or_status_,
      [&](const ValueWrapper &vw) -> const Status & { return OKAY_STATUS; },
      [](const Status &status) -> const Status & { return status; });
}

// Return from the enclosing function if the given status result is not
// ok. Otherwise, assign the value as the value of this expression. We don't
// really have a choice but to use the GNU statement expression extension here
// unless we pass the assignment target in too. We use the lambda because
// otherwise the statement expression doesn't preserve references.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define RETURN_OR_ASSIGN(status_value)                        \
  ({                                                          \
    auto &&evalutated_status_value =                          \
        std::forward<decltype(status_value)>(status_value);   \
    RETURN_IF_NOT_OK(evalutated_status_value.status());       \
    [&]() -> decltype(auto) {                                 \
      return std::forward<decltype(evalutated_status_value)>( \
                 evalutated_status_value)                     \
          .value();                                           \
    };                                                        \
  })()

}  // namespace resim
