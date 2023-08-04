// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>

namespace resim::testing {

// A simple class wrapping a test directory path which creates a unique test
// directory whenever it is constructed, and deletes it when destroyed.
class TestDirectoryRAII {
 public:
  // Default constructor. Creates a random directory.
  TestDirectoryRAII();

  // Destructor. Deletes the random directory.
  ~TestDirectoryRAII();

  // Delete copy and move constructors so we don't accidentally double delete
  // the folder.
  TestDirectoryRAII(const TestDirectoryRAII &) = delete;
  TestDirectoryRAII(TestDirectoryRAII &&) = delete;
  TestDirectoryRAII &operator=(const TestDirectoryRAII &) = delete;
  TestDirectoryRAII &operator=(TestDirectoryRAII &&) = delete;

  // Getter for the directory path.
  const std::filesystem::path &path() const;

  // Returns a unique path to a file with the given extension in the
  // given directory. This file is *not* created, however.
  // @param[in] The extension to put on this file, not including the
  //            dot. If empty, no dot is added.
  std::filesystem::path test_file_path(
      const std::string_view &extension = "") const;

 private:
  std::filesystem::path path_;
};

}  // namespace resim::testing
