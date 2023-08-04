// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace resim::third_party::glog {

// Ensure glog is installed and linked.
TEST(GlogTest, WriteSomeOutput) { LOG(INFO) << "Glog working successfully."; }

}  // namespace resim::third_party::glog
