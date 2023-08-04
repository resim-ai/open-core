# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = ["polynomial.hh"],
    deps = [],
)

cc_test(
    name = "polynomial_test",
    srcs = ["polynomial_test.cc"],
    deps = [
        ":polynomial",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "multivariate_gaussian",
    srcs = ["multivariate_gaussian.cc"],
    hdrs = ["multivariate_gaussian.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "multivariate_gaussian_test",
    srcs = ["multivariate_gaussian_test.cc"],
    deps = [
        ":multivariate_gaussian",
        ":sample_statistics",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sample_statistics",
    hdrs = ["sample_statistics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "sample_statistics_test",
    srcs = ["sample_statistics_test.cc"],
    deps = [
        ":sample_statistics",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "safe_integer_utils",
    srcs = ["safe_integer_utils.cc"],
    hdrs = ["safe_integer_utils.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
    ],
)

cc_test(
    name = "safe_integer_utils_test",
    srcs = ["safe_integer_utils_test.cc"],
    deps = [
        ":safe_integer_utils",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "vector_partition",
    hdrs = ["vector_partition.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "vector_partition_test",
    srcs = ["vector_partition_test.cc"],
    deps = [
        ":vector_partition",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "is_approx",
    hdrs = ["is_approx.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "is_approx_test",
    srcs = ["is_approx_test.cc"],
    deps = [
        ":is_approx",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
