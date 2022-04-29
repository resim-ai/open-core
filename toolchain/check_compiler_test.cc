#include "toolchain/check_compiler.hh"
#include <gtest/gtest.h>

namespace resim {
namespace toolchain {

TEST(CheckCompiler, query_data_returns) {
    const auto compiler_data = query_compiler_data();
    constexpr uint ZERO = 0;
    EXPECT_GT(compiler_data.cpp_standard.size(), ZERO);
}

TEST(CheckCompiler, verify_expected_compiler) {
    /* This test covers mainly the bazel toolchain config.
    The aim is to confirm that the expected compiler (clang 14) 
    and the expected cpp standard (2020) were used in the compilation.*/
    const auto compiler_data = query_compiler_data();
    EXPECT_NE(compiler_data.cpp_standard.find("2020"), std::string::npos);
    EXPECT_NE(compiler_data.compiler_version.find("Clang"), std::string::npos);
    EXPECT_NE(compiler_data.compiler_version.find("14"), std::string::npos);
}

} // namespace toolchain
} // namespace resim
