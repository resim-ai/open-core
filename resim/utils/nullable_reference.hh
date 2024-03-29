// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/assert/assert.hh"

namespace resim {

struct NullReferenceType {};

// This class represents a nullable reference to an object of type T.
template <class T>
class NullableReference {
 public:
  // Constructor from a reference to T
  explicit NullableReference(T &x);

  // This needs to allow implicit conversion since NullReferenceType
  // is interchangable with any null NullableReference<T>.
  // NOLINTNEXTLINE(google-explicit-constructor)
  NullableReference(NullReferenceType /* unused */);

  NullableReference() = default;

  // Check whether this NullableReference is populated.
  bool has_value() const;

  // Convenience conversion to bool which equals has_value().
  explicit operator bool() const;

  // Dereference the NullableReference
  T &operator*() const;

  // Access T's methods/members more conveniently
  T *operator->() const;

 private:
  static constexpr auto BAD_DEREFERENCE =
      "Can't dereference empty NullableReference!";
  T *x_{nullptr};
};

// Variable respresening a NULL NullableReference
const NullReferenceType null_reference;

template <typename T>
NullableReference<T>::NullableReference(T &x) : x_{&x} {}

template <typename T>
NullableReference<T>::NullableReference(NullReferenceType /* unused */) {}

template <typename T>
bool NullableReference<T>::has_value() const {
  return x_ != nullptr;
}

template <typename T>
NullableReference<T>::operator bool() const {
  return has_value();
}

template <typename T>
T &NullableReference<T>::operator*() const {
  REASSERT(has_value(), BAD_DEREFERENCE);
  return *x_;
}

template <typename T>
T *NullableReference<T>::operator->() const {
  REASSERT(has_value(), BAD_DEREFERENCE);
  return x_;
}

}  // namespace resim
