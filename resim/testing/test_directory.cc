// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/testing/test_directory.hh"

#include "resim/assert/assert.hh"
#include "resim/utils/uuid.hh"

namespace resim::testing {

namespace fs = std::filesystem;

TestDirectoryRAII::TestDirectoryRAII()
    : path_{fs::temp_directory_path() / UUID::new_uuid().to_string()} {
  const bool cond = fs::create_directory(path_);
  REASSERT(cond, "Failed to make directory!");
}

TestDirectoryRAII::~TestDirectoryRAII() { fs::remove_all(path_); }

const fs::path &TestDirectoryRAII::path() const { return path_; }

fs::path TestDirectoryRAII::test_file_path(
    const std::string_view &extension) const {
  if (extension.empty()) {
    return path_ / UUID::new_uuid().to_string();
  }
  return path_ / (UUID::new_uuid().to_string() + ".").append(extension);
}

}  // namespace resim::testing
