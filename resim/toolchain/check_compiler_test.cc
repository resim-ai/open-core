// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "resim/toolchain/check_compiler.hh"

namespace resim::toolchain {

TEST(CheckCompiler, QueryDataReturns) {
  const auto compiler_data = query_compiler_data();
  constexpr uint ZERO = 0;
  EXPECT_GT(compiler_data.cpp_standard.size(), ZERO);
}

TEST(CheckCompiler, VerifyExpectedCompiler) {
  /* This test covers mainly the bazel toolchain config.
  The aim is to confirm that the expected compiler (clang 14)
  and the expected cpp standard (2020) were used in the compilation.*/
  const auto compiler_data = query_compiler_data();
  EXPECT_NE(compiler_data.cpp_standard.find("2020"), std::string::npos);
  EXPECT_NE(compiler_data.compiler_version.find("Clang 19"), std::string::npos);
}

}  // namespace resim::toolchain
