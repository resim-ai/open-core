// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <utility>

namespace resim {

// This class can be used a setter and getter pair for a value in
// reference semantics. In other words, you can assign to it as if it
// were a non-const reference and it's implicitly convertible to the const
// reference that the getter returns.
template <typename Setter, typename Getter>
class SetterReference {
 public:
  SetterReference(Setter setter, Getter getter)
      : setter_{std::move(setter)},
        getter_{std::move(getter)} {}

  // Operator= which calls the given setter
  template <typename... Args>
  SetterReference<Setter, Getter> &operator=(Args &&...args) {
    setter_(std::forward<Args>(args)...);
    return *this;
  }

  // Implicit conversion to the type returned by the getter.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator decltype(std::declval<Getter>()())() const { return getter_(); }

 private:
  Setter setter_;
  Getter getter_;
};

}  // namespace resim
