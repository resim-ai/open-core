# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

exports_files(
    [
        "frame.hh",
        "liegroup.hh",
        "liegroup_concepts.hh",
        "se3.hh",
        "so3.hh",
        "frame.cc",
        "liegroup.cc",
        "liegroup_concepts.cc",
        "se3.cc",
        "so3.cc",
    ],
)

cc_library(
    name = "cross_matrix",
    srcs = ["cross_matrix.cc"],
    hdrs = ["cross_matrix.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "cross_matrix_test",
    srcs = ["cross_matrix_test.cc"],
    deps = [
        ":cross_matrix",
        ":liegroup_test_helpers",
        "//resim/math:is_approx",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "frame",
    hdrs = ["frame.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "frame_test",
    srcs = ["frame_test.cc"],
    deps = [
        ":frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "framed_vector",
    hdrs = ["framed_vector.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame",
        "//resim/assert",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "framed_vector_test",
    srcs = ["framed_vector_test.cc"],
    deps = [
        ":frame",
        ":framed_vector",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup",
    hdrs = ["liegroup.hh"],
    deps = [
        ":frame",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup_exp_diff",
    srcs = ["liegroup_exp_diff.cc"],
    hdrs = ["liegroup_exp_diff.hh"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_exp_diff_test",
    srcs = ["liegroup_exp_diff_test.cc"],
    deps = [
        ":liegroup_exp_diff",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup_test_helpers",
    testonly = 1,
    srcs = ["liegroup_test_helpers.cc"],
    hdrs = ["liegroup_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":se3",
        ":so3",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/utils:type",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_test_helpers_test",
    srcs = ["liegroup_test_helpers_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        ":so3",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_test",
    srcs = ["liegroup_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        ":so3",
        "//resim/math:is_approx",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "se3",
    srcs = ["se3.cc"],
    hdrs = ["se3.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":cross_matrix",
        ":frame",
        ":liegroup",
        ":liegroup_concepts",
        ":liegroup_exp_diff",
        ":so3",
        "//resim/assert",
        "//resim/math:is_approx",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "se3_test",
    srcs = ["se3_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "so3",
    srcs = ["so3.cc"],
    hdrs = ["so3.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":cross_matrix",
        ":frame",
        ":framed_vector",
        ":liegroup",
        ":liegroup_concepts",
        ":liegroup_exp_diff",
        "//resim/assert",
        "//resim/math:is_approx",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "so3_test",
    srcs = ["so3_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":so3",
        "//resim/math:is_approx",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup_concepts",
    hdrs = ["liegroup_concepts.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup",
    ],
)

cc_test(
    name = "liegroup_concepts_test",
    srcs = ["liegroup_concepts_test.cc"],
    deps = [
        ":liegroup_concepts",
        ":se3",
        ":so3",
        "@com_google_googletest//:gtest_main",
    ],
)
