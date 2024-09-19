# Copyright 2023 ReSim, Inc.
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
        "//resim/transforms:se3",
    ],
)

cc_test(
    name = "state_test",
    srcs = ["state_test.cc"],
    deps = [
        ":state",
        "//resim/math:is_approx",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inertia",
    srcs = ["inertia.cc"],
    hdrs = ["inertia.hh"],
    deps = [
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "inertia_test",
    srcs = ["inertia_test.cc"],
    deps = [
        ":inertia",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "dynamics",
    srcs = ["dynamics.cc"],
    hdrs = ["dynamics.hh"],
    deps = [
        ":inertia",
        ":state",
        "//resim/assert",
        "//resim/dynamics",
        "//resim/math:vector_partition",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "dynamics_test",
    srcs = ["dynamics_test.cc"],
    deps = [
        ":dynamics",
        ":inertia",
        ":state",
        "//resim/assert",
        "//resim/dynamics:forward_euler",
        "//resim/time:sample_interval",
        "//resim/time:timestamp",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/utils:nullable_reference",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gravity",
    srcs = ["gravity.cc"],
    hdrs = ["gravity.hh"],
    deps = [
        ":state",
        "//resim/assert",
        "//resim/dynamics:constants",
        "//resim/dynamics:controller",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "//resim/utils:nullable_reference",
        "@au//au",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "gravity_test",
    srcs = ["gravity_test.cc"],
    deps = [
        ":gravity",
        ":state",
        "//resim/dynamics:controller",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "@au//au",
        "@com_google_googletest//:gtest_main",
    ],
)
