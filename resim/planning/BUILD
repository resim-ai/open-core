# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "dynamics",
    hdrs = ["dynamics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "cost_function",
    hdrs = ["cost_function.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "cost_building_blocks",
    hdrs = ["cost_building_blocks.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "cost_building_blocks_test",
    srcs = ["cost_building_blocks_test.cc"],
    deps = [
        ":cost_building_blocks",
        "//resim/math:is_approx",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "ilqr",
    hdrs = ["ilqr.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":cost_function",
        ":dynamics",
        "//resim/assert",
        "//resim/utils:double_buffer",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "ilqr_test",
    size = "small",
    srcs = ["ilqr_test.cc"],
    deps = [
        ":ilqr",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "cost_function_registry",
    hdrs = ["cost_function_registry.hh"],
    deps = [
        "//resim/planning:cost_function",
        "//resim/utils:nullable_reference",
    ],
)

cc_test(
    name = "cost_function_registry_test",
    srcs = ["cost_function_registry_test.cc"],
    deps = [
        ":cost_function_registry",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
