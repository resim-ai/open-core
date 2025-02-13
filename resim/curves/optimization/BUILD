# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "two_jet_tangent_space",
    srcs = ["two_jet_tangent_space.cc"],
    hdrs = ["two_jet_tangent_space.hh"],
    deps = [
        "//resim/curves:two_jet",
        "//resim/math:vector_partition",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "two_jet_tangent_space_test",
    srcs = ["two_jet_tangent_space_test.cc"],
    deps = [
        ":two_jet_tangent_space",
        "//resim/curves:two_jet",
        "//resim/curves:two_jet_test_helpers",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "t_curve_differential",
    srcs = ["t_curve_differential.cc"],
    hdrs = ["t_curve_differential.hh"],
    deps = [
        ":two_jet_tangent_space",
        "//resim/assert",
        "//resim/curves:quintic_poly_coeffs",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/math:vector_partition",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "t_curve_differential_test",
    srcs = ["t_curve_differential_test.cc"],
    deps = [
        ":t_curve_differential",
        ":two_jet_tangent_space",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/curves:two_jet_test_helpers",
        "//resim/math:is_approx",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "pose_error_model",
    srcs = ["pose_error_model.cc"],
    hdrs = ["pose_error_model.hh"],
    deps = [
        ":t_curve_differential",
        "//resim/transforms:se3",
        "//resim/curves:t_curve",
        "//resim/math:gauss_newton_optimizer",
        "@libeigen//:eigen",

    ],
)
