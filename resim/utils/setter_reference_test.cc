// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/setter_reference.hh"

#include <gtest/gtest.h>

namespace resim {

namespace {

class TestObject {
 public:
  void set_x(int x) { x_ = x; }
  int x() const { return x_; };

 private:
  int x_ = 0;
};
}  // namespace

TEST(SetterReferenceTest, TestSetterReference) {
  // SETUP
  TestObject obj;

  // ACTION
  SetterReference x_ref{
      [&obj](int val) { return obj.set_x(val); },
      [&obj]() { return obj.x(); }};

  constexpr int NEW_VALUE = 5;
  x_ref = NEW_VALUE;

  // VERIFICATION
  EXPECT_EQ(x_ref, NEW_VALUE);
}
}  // namespace resim
