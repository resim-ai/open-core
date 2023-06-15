
#include "resim/testing/test_directory.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace resim::testing {

namespace {
namespace fs = std::filesystem;

bool ends_with(const std::string &string, const std::string_view &suffix) {
  return string.compare(string.size() - suffix.size(), suffix.size(), suffix) ==
         0;
}
}  // namespace

TEST(TestDirectoryTest, TestTestDirectoryExists) {
  // SETUP
  fs::path test_directory_path;
  {
    // ACTION
    const TestDirectoryRAII test_directory;

    // VERIFICATION
    EXPECT_TRUE(fs::is_directory(test_directory.path()));
    test_directory_path = test_directory.path();
  }
  // Check that the directory has been deleted when the resource is released
  EXPECT_FALSE(fs::is_directory(test_directory_path));
}

TEST(TestDirectoryTest, TestTestDirectoryFileDeletes) {
  // SETUP
  fs::path test_file_path;
  {
    // ACTION
    const TestDirectoryRAII test_directory;
    test_file_path = test_directory.test_file_path();
    std::ofstream test_file{test_file_path};
    test_file << "Hello, World!" << std::endl;
    test_file.close();
    ASSERT_TRUE(fs::is_regular_file(test_file_path));
  }
  // Check that the file has been deleted when the resource is
  // released
  EXPECT_FALSE(fs::exists(test_file_path));
}

TEST(TestDirectoryTest, TestTestDirectoryUnique) {
  // SETUP
  const TestDirectoryRAII test_directory_a;

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // ACTION
    const TestDirectoryRAII test_directory_b;

    // VERIFICATION
    EXPECT_NE(test_directory_a.path(), test_directory_b.path());
  }
}

TEST(TestDirectoryTest, TestTestFilePath) {
  // SETUP
  using fs::path;
  const TestDirectoryRAII test_directory;

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // ACTION
    constexpr std::string_view EXTENSION = "foo";
    const path path_a{test_directory.test_file_path()};
    const path path_b{test_directory.test_file_path()};
    const path path_with_ext_a{test_directory.test_file_path(EXTENSION)};
    const path path_with_ext_b{test_directory.test_file_path(EXTENSION)};

    // VERIFICATION
    EXPECT_NE(path_a, path_b);
    EXPECT_NE(path_a.string().back(), '.');

    EXPECT_NE(path_with_ext_a, path_with_ext_b);
    // Check that we have the expected extension
    std::string expected_suffix{"."};
    expected_suffix += EXTENSION;
    EXPECT_TRUE(ends_with(path_with_ext_a, expected_suffix));
  }
}

}  // namespace resim::testing
