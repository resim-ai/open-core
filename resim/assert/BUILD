# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

exports_files(
    ["assert.hh"],
)

cc_library(
    name = "assert",
    srcs = ["assert.cc"],
    hdrs = ["assert.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
        "@fmt",
    ],
)

cc_test(
    name = "assert_test",
    srcs = ["assert_test.cc"],
    deps = [
        ":assert",
        "@com_google_googletest//:gtest_main",
    ],
)
