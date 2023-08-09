# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_test")

exports_files(
    glob(["*.BUILD"]),
    visibility = ["//visibility:public"],
)

cc_test(
    name = "ros2_test",
    srcs = ["ros2_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@ros2_rcpputils//:rcpputils",
    ],
)
