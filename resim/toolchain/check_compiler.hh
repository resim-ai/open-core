// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>

namespace resim::toolchain {

struct CompilerData {
  std::string cpp_standard;
  std::string compiler_version;
};

CompilerData query_compiler_data() {
  return CompilerData{std::to_string(__cplusplus), __VERSION__};
}

}  // namespace resim::toolchain
