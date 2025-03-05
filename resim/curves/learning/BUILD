# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "two_jet_mean",
    srcs = ["two_jet_mean.cc"],
    hdrs = ["two_jet_mean.hh"],
    deps = [
        "//resim/assert",
        "//resim/curves:two_jet",
        "//resim/curves/optimization:two_jet_tangent_space",
        "//resim/transforms:se3",
        "//resim/utils:status_value",
    ],
)

cc_test(
    name = "two_jet_mean_test",
    srcs = ["two_jet_mean_test.cc"],
    deps = [
        ":two_jet_mean",
        "//resim/curves:two_jet_test_helpers",
        "//resim/curves/optimization:two_jet_tangent_space",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)
