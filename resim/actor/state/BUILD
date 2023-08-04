# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "rigid_body_state",
    srcs = ["rigid_body_state.cc"],
    hdrs = ["rigid_body_state.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/curves:two_jet",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "rigid_body_state_test",
    srcs = ["rigid_body_state_test.cc"],
    deps = [
        ":rigid_body_state",
        "//resim/curves:two_jet",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":rigid_body_state",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = ["trajectory_test.cc"],
    deps = [
        ":rigid_body_state",
        ":trajectory",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/testing:random_matrix",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "observable_state",
    hdrs = ["observable_state.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor:actor_id",
        "//resim/actor/state:rigid_body_state",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
    ],
)
