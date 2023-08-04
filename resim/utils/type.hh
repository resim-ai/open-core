// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

//
// type.hh
//
// This file implements types as objects similar to boost Hana's
// boost::hana::type template does. What's the point of doing this? The primary
// benefit is that it allows passing types naturally into functions, function
// templates, or auto-type-deduced lambdas. These functions can also return type
// objects and can be declared constexpr also, making it easy to write
// metafunctions using normal C++ function syntax as opposed to relying on
// write-only partial struct template specialization code. Restrictions on
// function partial specialization can also be circumvented by using overloading
// and template argument deduction instead. See the following (in particular the
// section on "Types as objects") for more info:
// https://www.boost.org/doc/libs/1_80_0/libs/hana/doc/html/index.html#tutorial-description
//
// A brief explanation of the templates defined herein and how they work
// together:
//
// BasicType is simply an empty template of type T. Users should use this type
// for all template pattern matching instead of Type<T> because Type<T> is a
// dependent type. E.g. you should probably never write something like:
//
// template <typename T>
// void my_func(Type<T>) {
//   // ...
// }
//
// Unless they intend to specify T *explicitly* because this is equivalent to:
//
// template <typename T>
// void my_func(detail::TypeADLGuard<T>::Type) {
//   // ...
// }
//
// And the type T *will not* be deducible during template argument
// deduction. Instead a user should do:
//
// template <typename T>
// void my_func(BasicType<T>) {
//   // ...
// }
//
// If they want T to be deducible. Why then do we need Type<T> at all? Can't we
// just always use BasicType<T>? The rub is that Argument Dependent Lookup (ADL)
// will bring in all overloads of my_func() in T's namespace if we pass a
// BasicType<T> object to my_func(). Since T may be declared in a client library
// or elsewhere, it can be very dangerous to simply bring in all functions
// called my_func() that might be defined therein. This is *why* we make Type<T>
// a depenedent type in the first place. The general usage pattern is then:
//
// // Library code
// template <typename T>
// void my_func(BasicType<T>) { // Defined with BasicType<T> for pattern
// matching
//   // ...
// }
//
// void my_func(BasicType<int>) { // Overload specializing for ints
//   // ...
// }
//
// // Client code
// my_func(Type<ClientType>{});
//
// Since the pattern of Type<T>{} is so common, we provide the variable template
// TypeC<T> to make it slightly more readable.
//
// my_func(TypeC<ClientType>);
//
// Aliases are defined within both BasicType and Type to make it possible to
// "unwrap" the types for compatibility with existing metafunctions and access
// to their static members.
//
#pragma once

namespace resim {

template <typename T>
struct BasicType {
  using type = T;
};

namespace detail {
template <typename T>
struct TypeADLGuard {
  struct Type : BasicType<T> {
    using type = T;
  };
};
}  // namespace detail

template <typename T>
using Type = typename detail::TypeADLGuard<T>::Type;

template <typename T>
constexpr Type<T> TypeC{};

}  // namespace resim
