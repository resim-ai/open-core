#pragma once

#include <string>

namespace resim {
namespace toolchain {

struct CompilerData {
  std::string cpp_standard;
  std::string compiler_version;
};

const CompilerData query_compiler_data() {
  return CompilerData{std::to_string(__cplusplus), __VERSION__};
}

}  // namespace toolchain
}  // namespace resim
