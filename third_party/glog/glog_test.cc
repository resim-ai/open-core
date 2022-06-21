#include <glog/logging.h>
#include <gtest/gtest.h>

namespace resim::third_party::glog {

// Ensure glog is installed and linked.
TEST(GlogTest, WriteSomeOutput) { LOG(INFO) << "Glog working successfully."; }

}  // namespace resim::third_party::glog
