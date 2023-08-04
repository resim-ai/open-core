// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

namespace resim {

// This class is a simple wrapper around a reference that one can use in
// function signatures to make it impossible for arguments to be modified
// without clients expecting it. In other words, when passing an argument to a
// function taking an InOut<T>, a user must wrap their object in an InOut first
// making it obvious that that object may be modified. For example compare the
// two approaches:
//
// void foo(int &value) { ++value; }
// void bar(InOut<int> value) { ++(*value); }
//
// // Client code:
// int a = 5;
// foo(a);  // a is incremented without a user having any indication that this
//          // could happen
//
// bar(InOut{a}); // User can tell that a may be modified by bar().
template <class T>
class InOut {
 public:
  // Constructor from a reference to T
  explicit InOut(T &x) : x_{x} {}

  // Dereference the InOut
  T &operator*() { return x_; }

  // Access T's methods/members more conveniently
  T *operator->() { return &x_; }

 private:
  explicit InOut(T *x);
  T &x_;
};

}  // namespace resim
