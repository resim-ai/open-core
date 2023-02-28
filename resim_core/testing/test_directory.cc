
#include "resim_core/testing/test_directory.hh"

#include <glog/logging.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::testing {

namespace fs = std::filesystem;

TestDirectoryRAII::TestDirectoryRAII()
    : path_{fs::temp_directory_path() / UUID::new_uuid().to_string()} {
  const bool cond = fs::create_directory(path_);
  REASSERT(cond, "Failed to make directory!");
}

TestDirectoryRAII::~TestDirectoryRAII() {
  const bool cond = fs::remove_all(path_) != 0U;

  // We don't use REASSERT because we shouldn't throw in a destructor.
  CHECK(cond) << "Failed to remove directory!";
}

const fs::path &TestDirectoryRAII::path() const { return path_; }

fs::path TestDirectoryRAII::test_file_path(
    const std::string_view &extension) const {
  if (extension.empty()) {
    return path_ / UUID::new_uuid().to_string();
  }
  return path_ / (UUID::new_uuid().to_string() + ".").append(extension);
}

}  // namespace resim::testing
