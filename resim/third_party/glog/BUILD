# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "glog_test",
    srcs = ["glog_test.cc"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)
