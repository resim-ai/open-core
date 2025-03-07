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

cc_library(
    name = "t_curve_distribution",
    hdrs = ["t_curve_distribution.hh"],
    deps = [
        "//resim/curves:t_curve",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "learn_t_curve_distribution",
    srcs = ["learn_t_curve_distribution.cc"],
    hdrs = ["learn_t_curve_distribution.hh"],
    deps = [
        ":t_curve_distribution",
        ":two_jet_mean",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/optimization:two_jet_tangent_space",
        "//resim/transforms:se3",
        "//resim/utils:status_value",
    ],
)

cc_test(
    name = "learn_t_curve_distribution_test",
    srcs = ["learn_t_curve_distribution_test.cc"],
    deps = [
        ":learn_t_curve_distribution",
        "//resim/curves:sample_t_curve",
        "//resim/math:is_approx",
        "//resim/time:sample_interval",
        "//resim/visualization:save_visualization_log",
        "@com_google_googletest//:gtest_main",
    ],
)
