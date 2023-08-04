// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/inout.hh"

#include <gtest/gtest.h>

#include <string>

namespace resim {

namespace {

// A simple helper to set the given target with a new value.
// @param[in] value - The value to assign to the target.
// @param[inout] target - The target to assign to
template <class T>
void set_target(const T &value, InOut<T> target) {
  *target = value;
}

// A simple test struct we can use to test operator->() behavior with InOut.
struct TestStruct {
  void set_x(const int new_x) { x = new_x; }
  int x = 0;
};

// A simple function which uses the arrow operator to call a method from
// TestStruct from an InOut.
void set_test_struct_x(const int new_x, InOut<TestStruct> target) {
  target->set_x(new_x);
}

}  // namespace

// Test that we can set ints with InOut.
TEST(InOutTest, TestInOutInt) {
  // SETUP
  constexpr int VALUE = 3;
  int target = 0;

  // ACTION
  set_target(VALUE, InOut{target});

  // VERIFICATION
  EXPECT_EQ(target, VALUE);
}

// Test that we can set doubles with InOut.
TEST(InOutTest, TestInOutDouble) {
  // SETUP
  constexpr double VALUE = 3.0;
  double target = 0;

  // ACTION
  set_target(VALUE, InOut{target});

  // VERIFICATION
  EXPECT_EQ(target, VALUE);
}

// Test that we can set strings with InOut.
TEST(InOutTest, TestInOutString) {
  // SETUP
  const std::string value{"my_string"};
  std::string target;

  // ACTION
  set_target(value, InOut{target});

  // VERIFICATION
  EXPECT_EQ(target, value);
}

// Test that the arrow operator works with InOut.
TEST(InOutTest, TestArrowOperator) {
  // SETUP
  TestStruct target;
  const int value = target.x + 1;

  // ACTION
  set_test_struct_x(value, InOut{target});

  // VERIFICATION
  EXPECT_EQ(target.x, value);
}

}  // namespace resim
