# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/math:vector_partition",
        "//resim/transforms:so3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "state_test",
    srcs = ["state_test.cc"],
    deps = [
        ":state",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)